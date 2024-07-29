/*
 * Copyright (c) 2020 PHYTEC Messtechnik GmbH
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(modbus, CONFIG_MODBUS_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <modbus_internal.h>

#define DT_DRV_COMPAT zephyr_modbus_serial

#define MB_RTU_DEFINE_GPIO_CFG(inst, prop)				\
	static struct gpio_dt_spec prop##_cfg_##inst = {		\
		.port = DEVICE_DT_GET(DT_INST_PHANDLE(inst, prop)),	\
		.pin = DT_INST_GPIO_PIN(inst, prop),			\
		.dt_flags = DT_INST_GPIO_FLAGS(inst,  prop),		\
	};

#define MB_RTU_DEFINE_GPIO_CFGS(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, de_gpios),		\
		    (MB_RTU_DEFINE_GPIO_CFG(inst, de_gpios)), ())	\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, re_gpios),		\
		    (MB_RTU_DEFINE_GPIO_CFG(inst, re_gpios)), ())

DT_INST_FOREACH_STATUS_OKAY(MB_RTU_DEFINE_GPIO_CFGS)

#define MB_RTU_ASSIGN_GPIO_CFG(inst, prop)			\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),		\
		    (&prop##_cfg_##inst), (NULL))

#define MODBUS_DT_GET_SERIAL_DEV(inst) {			\
		.dev = DEVICE_DT_GET(DT_INST_PARENT(inst)),	\
		.de = MB_RTU_ASSIGN_GPIO_CFG(inst, de_gpios),	\
		.re = MB_RTU_ASSIGN_GPIO_CFG(inst, re_gpios),	\
	},

#ifdef CONFIG_MODBUS_SERIAL
static struct modbus_serial_config modbus_serial_cfg[] = {
	DT_INST_FOREACH_STATUS_OKAY(MODBUS_DT_GET_SERIAL_DEV)
};
#endif

#define MODBUS_DT_GET_DEV(inst) {				\
		.iface_name = DEVICE_DT_NAME(DT_DRV_INST(inst)),\
		.cfg = &modbus_serial_cfg[inst],		\
	},

#define DEFINE_MODBUS_RAW_ADU(x, _) {				\
		.iface_name = "RAW_"#x,				\
		.rawcb.raw_tx_cb = NULL,				\
		.mode = MODBUS_MODE_RAW,			\
	}

static struct modbus_context mb_ctx_tbl[] = {
	DT_INST_FOREACH_STATUS_OKAY(MODBUS_DT_GET_DEV)
#ifdef CONFIG_MODBUS_RAW_ADU
	LISTIFY(CONFIG_MODBUS_NUMOF_RAW_ADU, DEFINE_MODBUS_RAW_ADU, (,), _)
#endif
};

/* liteon start */
static void modbus_trans_timeout(struct k_timer *timer)
{
	struct modbus_context *ctx = k_timer_user_data_get(timer);

	ctx->trans_status = MODBUS_TRANS_TIMEOUT;
	ctx->trans_result = -ETIMEDOUT;
	LOG_DBG("Submit transparent timeout work");
	k_work_submit(&ctx->server_work);
}
/* liteon end */

static void modbus_rx_handler(struct k_work *item)
{
	struct modbus_context *ctx;

	ctx = CONTAINER_OF(item, struct modbus_context, server_work);

	/* liteon start */
	if (ctx->trans_status == MODBUS_TRANS_WAIT_NOTIFY) {
		LOG_ERR("Another transparent operation is pending");
		return;
	}

	if (ctx->trans_status == MODBUS_TRANS_DONE) {
		goto trans_resume;
	}
	/* liteon end */

	switch (ctx->mode) {
	case MODBUS_MODE_RTU:
	case MODBUS_MODE_ASCII:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL)) {
			modbus_serial_rx_disable(ctx);
			ctx->rx_adu_err = modbus_serial_rx_adu(ctx);
		}
		break;
	case MODBUS_MODE_RAW:
		if (IS_ENABLED(CONFIG_MODBUS_RAW_ADU)) {
			ctx->rx_adu_err = modbus_raw_rx_adu(ctx);
		}
		break;
	default:
		LOG_ERR("Unknown MODBUS mode");
		return;
	}

trans_resume:

	if (ctx->client == true) {
		k_sem_give(&ctx->client_wait_sem);

	} else if (IS_ENABLED(CONFIG_MODBUS_SERVER)) {

		bool respond;
		/* liteon start */
		if (!ctx->pmm_server) {
			respond = modbus_server_handler(ctx);
		} else {
			respond = modbus_server_handler_pmm(ctx);
		}

		/* inturrupt point for waiting transparent operation */
		if (ctx->trans_status == MODBUS_TRANS_WAIT_NOTIFY) {
			LOG_DBG("Interrupt modbus_rx_handler for waiting transparent operation finished");
			k_timer_user_data_set(&ctx->trans_timeout_timer, ctx);
			k_timer_start(&ctx->trans_timeout_timer, ctx->trans_timeout, K_NO_WAIT);

			return;
		}
		/* liteon end */

		if (respond) {
			modbus_tx_adu(ctx);
		} else {
			LOG_DBG("Server has dropped frame");
		}

		switch (ctx->mode) {
		case MODBUS_MODE_RTU:
		case MODBUS_MODE_ASCII:
			if (IS_ENABLED(CONFIG_MODBUS_SERIAL) &&
			    respond == false) {
				modbus_serial_rx_enable(ctx);
			}
			break;
		default:
			break;
		}

		/* litone start */
		ctx->trans_status = MODBUS_TRANS_NONE;
		/* litone end */
	}
}

void modbus_tx_adu(struct modbus_context *ctx)
{
	switch (ctx->mode) {
	case MODBUS_MODE_RTU:
	case MODBUS_MODE_ASCII:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL) &&
		    modbus_serial_tx_adu(ctx)) {
			LOG_ERR("Unsupported MODBUS serial mode");
		}
		break;
	case MODBUS_MODE_RAW:
		if (IS_ENABLED(CONFIG_MODBUS_RAW_ADU) &&
		    modbus_raw_tx_adu(ctx)) {
			LOG_ERR("Unsupported MODBUS raw mode");
		}
		break;
	default:
		LOG_ERR("Unknown MODBUS mode");
	}
}

int modbus_tx_wait_rx_adu(struct modbus_context *ctx)
{
	modbus_tx_adu(ctx);

	if (k_sem_take(&ctx->client_wait_sem, K_USEC(ctx->rxwait_to)) != 0) {
		LOG_WRN("Client wait-for-RX timeout");
		return -ETIMEDOUT;
	}

	return ctx->rx_adu_err;
}

struct modbus_context *modbus_get_context(const uint8_t iface)
{
	struct modbus_context *ctx;

	if (iface >= ARRAY_SIZE(mb_ctx_tbl)) {
		LOG_ERR("Interface %u not available", iface);
		return NULL;
	}

	ctx = &mb_ctx_tbl[iface];

	if (!atomic_test_bit(&ctx->state, MODBUS_STATE_CONFIGURED)) {
		LOG_ERR("Interface not configured");
		return NULL;
	}

	return ctx;
}

int modbus_iface_get_by_ctx(const struct modbus_context *ctx)
{
	for (int i = 0; i < ARRAY_SIZE(mb_ctx_tbl); i++) {
		if (&mb_ctx_tbl[i] == ctx) {
			return i;
		}
	}

	return -ENODEV;
}

int modbus_iface_get_by_name(const char *iface_name)
{
	for (int i = 0; i < ARRAY_SIZE(mb_ctx_tbl); i++) {
		if (strcmp(iface_name, mb_ctx_tbl[i].iface_name) == 0) {
			return i;
		}
	}

	return -ENODEV;
}

static struct modbus_context *modbus_init_iface(const uint8_t iface)
{
	struct modbus_context *ctx;

	if (iface >= ARRAY_SIZE(mb_ctx_tbl)) {
		LOG_ERR("Interface %u not available", iface);
		return NULL;
	}

	ctx = &mb_ctx_tbl[iface];

	if (atomic_test_and_set_bit(&ctx->state, MODBUS_STATE_CONFIGURED)) {
		LOG_ERR("Interface already used");
		return NULL;
	}

	k_mutex_init(&ctx->iface_lock);
	k_sem_init(&ctx->client_wait_sem, 0, 1);
	k_work_init(&ctx->server_work, modbus_rx_handler);

	return ctx;
}

static int modbus_user_fc_init(struct modbus_context *ctx, struct modbus_iface_param param)
{
	sys_slist_init(&ctx->user_defined_cbs);
	LOG_DBG("Initializing user-defined function code support.");

	return 0;
}

int modbus_init_server(const int iface, struct modbus_iface_param param)
{
	struct modbus_context *ctx = NULL;
	int rc = 0;

	if (!IS_ENABLED(CONFIG_MODBUS_SERVER)) {
		LOG_ERR("Modbus server support is not enabled");
		rc = -ENOTSUP;
		goto init_server_error;
	}

	if (param.server.user_cb == NULL) {
		LOG_ERR("User callbacks should be available");
		rc = -EINVAL;
		goto init_server_error;
	}

	ctx = modbus_init_iface(iface);
	if (ctx == NULL) {
		rc = -EINVAL;
		goto init_server_error;
	}

	ctx->client = false;
	/* liteon start */
	ctx->pmm_server = false;
	ctx->trans_status = MODBUS_TRANS_NONE;
	ctx->trans_result = 0;
	ctx->trans_timeout = K_NO_WAIT;
	/* liteon end */

	if (modbus_user_fc_init(ctx, param) != 0) {
		LOG_ERR("Failed to init MODBUS user defined function codes");
		rc = -EINVAL;
		goto init_server_error;
	}

	switch (param.mode) {
	case MODBUS_MODE_RTU:
	case MODBUS_MODE_ASCII:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL) &&
		    modbus_serial_init(ctx, param) != 0) {
			LOG_ERR("Failed to init MODBUS over serial line");
			rc = -EINVAL;
			goto init_server_error;
		}
		break;
	case MODBUS_MODE_RAW:
		if (IS_ENABLED(CONFIG_MODBUS_RAW_ADU) &&
		    modbus_raw_init(ctx, param) != 0) {
			LOG_ERR("Failed to init MODBUS raw ADU support");
			rc = -EINVAL;
			goto init_server_error;
		}
		break;
	default:
		LOG_ERR("Unknown MODBUS mode");
		rc = -ENOTSUP;
		goto init_server_error;
	}

	ctx->unit_id = param.server.unit_id;
	sys_slist_init(&ctx->pmm_ids);
	ctx->mbs_user_cb = param.server.user_cb;
	if (IS_ENABLED(CONFIG_MODBUS_FC08_DIAGNOSTIC)) {
		modbus_reset_stats(ctx);
	}

	LOG_DBG("Modbus interface %s initialized", ctx->iface_name);

	return 0;

init_server_error:
	if (ctx != NULL) {
		atomic_clear_bit(&ctx->state, MODBUS_STATE_CONFIGURED);
	}

	return rc;
}

int modbus_init_server_pmm(const int iface, struct modbus_iface_param param)
{
	struct modbus_context *ctx = NULL;
	int rc = 0;

	if (!IS_ENABLED(CONFIG_MODBUS_SERVER)) {
		LOG_ERR("Modbus server support is not enabled");
		rc = -ENOTSUP;
		goto init_server_error;
	}

	if (param.server.user_cb == NULL) {
		LOG_ERR("User callbacks should be available");
		rc = -EINVAL;
		goto init_server_error;
	}

	ctx = modbus_init_iface(iface);
	if (ctx == NULL) {
		rc = -EINVAL;
		goto init_server_error;
	}

	ctx->client = false;
	/* liteon start */
	ctx->pmm_server = true;
	ctx->trans_status = MODBUS_TRANS_NONE;
	ctx->trans_result = 0;
	k_timer_init(&ctx->trans_timeout_timer, modbus_trans_timeout, NULL);
	/* liteon end */

	if (modbus_user_fc_init(ctx, param) != 0) {
		LOG_ERR("Failed to init MODBUS user defined function codes");
		rc = -EINVAL;
		goto init_server_error;
	}

	switch (param.mode) {
	case MODBUS_MODE_RTU:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL) &&
		    modbus_serial_init(ctx, param) != 0) {
			LOG_ERR("Failed to init MODBUS over serial line");
			rc = -EINVAL;
			goto init_server_error;
		}
		break;
	default:
		LOG_ERR("Only support dynamic id function under RTU MODBUS mode");
		rc = -ENOTSUP;
		goto init_server_error;
	}

	ctx->unit_id = 0;
	/* liteon start */
	sys_slist_init(&ctx->pmm_ids);
	ctx->mbs_pmm_user_cb = param.pmm_server.pmm_user_cb;
	ctx->trans_timeout = K_USEC(param.pmm_server.trans_timeout);
	/* liteon end */
	if (IS_ENABLED(CONFIG_MODBUS_FC08_DIAGNOSTIC)) {
		modbus_reset_stats(ctx);
	}

	LOG_DBG("Modbus interface %s initialized", ctx->iface_name);

	return 0;

init_server_error:
	if (ctx != NULL) {
		atomic_clear_bit(&ctx->state, MODBUS_STATE_CONFIGURED);
	}

	return rc;
}

int modbus_register_user_fc(const int iface, struct modbus_custom_fc *custom_fc)
{
	struct modbus_context *ctx = modbus_get_context(iface);

	if (!custom_fc) {
		LOG_ERR("Provided function code handler was NULL");
		return -EINVAL;
	}

	if (custom_fc->fc & BIT(7)) {
		LOG_ERR("Function codes must have MSB of 0");
		return -EINVAL;
	}

	custom_fc->excep_code = MODBUS_EXC_NONE;

	LOG_DBG("Registered new custom function code %d", custom_fc->fc);
	sys_slist_append(&ctx->user_defined_cbs, &custom_fc->node);

	return 0;
}

int modbus_init_client(const int iface, struct modbus_iface_param param)
{
	struct modbus_context *ctx = NULL;
	int rc = 0;

	if (!IS_ENABLED(CONFIG_MODBUS_CLIENT)) {
		LOG_ERR("Modbus client support is not enabled");
		rc = -ENOTSUP;
		goto init_client_error;
	}

	ctx = modbus_init_iface(iface);
	if (ctx == NULL) {
		rc = -EINVAL;
		goto init_client_error;
	}

	ctx->client = true;
	/* liteon start */
	ctx->pmm_server = false;
	ctx->trans_status = MODBUS_TRANS_NONE;
	ctx->trans_result = 0;
	ctx->trans_timeout = K_NO_WAIT;
	/* liteon end */

	switch (param.mode) {
	case MODBUS_MODE_RTU:
	case MODBUS_MODE_ASCII:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL) &&
		    modbus_serial_init(ctx, param) != 0) {
			LOG_ERR("Failed to init MODBUS over serial line");
			rc = -EINVAL;
			goto init_client_error;
		}
		break;
	case MODBUS_MODE_RAW:
		if (IS_ENABLED(CONFIG_MODBUS_RAW_ADU) &&
		    modbus_raw_init(ctx, param) != 0) {
			LOG_ERR("Failed to init MODBUS raw ADU support");
			rc = -EINVAL;
			goto init_client_error;
		}
		break;
	default:
		LOG_ERR("Unknown MODBUS mode");
		rc = -ENOTSUP;
		goto init_client_error;
	}

	ctx->unit_id = 0;
	ctx->mbs_user_cb = NULL;
	ctx->rxwait_to = param.rx_timeout;

	return 0;

init_client_error:
	if (ctx != NULL) {
		atomic_clear_bit(&ctx->state, MODBUS_STATE_CONFIGURED);
	}

	return rc;
}

int modbus_disable(const uint8_t iface)
{
	struct modbus_context *ctx;
	struct k_work_sync work_sync;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return -EINVAL;
	}

	switch (ctx->mode) {
	case MODBUS_MODE_RTU:
	case MODBUS_MODE_ASCII:
		if (IS_ENABLED(CONFIG_MODBUS_SERIAL)) {
			modbus_serial_disable(ctx);
		}
		break;
	case MODBUS_MODE_RAW:
		break;
	default:
		LOG_ERR("Unknown MODBUS mode");
	}

	k_work_cancel_sync(&ctx->server_work, &work_sync);
	ctx->rxwait_to = 0;
	ctx->unit_id = 0;
	ctx->mbs_user_cb = NULL;
	/* liteon start */
	if (ctx->pmm_server) {
		ctx->trans_status = MODBUS_TRANS_NONE;
		ctx->trans_result = 0;
		_modbus_unregister_all_pmm_id(ctx);
		ctx->pmm_server = false;
	}
	/* liteon end */

	atomic_clear_bit(&ctx->state, MODBUS_STATE_CONFIGURED);

	LOG_INF("Modbus interface %u disabled", iface);

	return 0;
}

/* liteon start */
struct modbus_pmm_id *_modbus_find_pmm_id(struct modbus_context *ctx, uint8_t id)
{
	struct modbus_pmm_id *pmm_id;

	/*
	 * Find the pmm_id st with the specified unit id
	 */
	k_mutex_lock(&ctx->iface_lock, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&ctx->pmm_ids, pmm_id, node)
	if (pmm_id->id == id) {
		k_mutex_unlock(&ctx->iface_lock);
		return pmm_id;
	}
	k_mutex_unlock(&ctx->iface_lock);

	return NULL;
}

void _modbus_unregister_all_pmm_id(struct modbus_context *ctx)
{
	struct modbus_pmm_id *pmm_id, *next;

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&ctx->pmm_ids, pmm_id, next, node) {
		sys_slist_remove(&ctx->pmm_ids, NULL, &pmm_id->node);
		k_free(pmm_id);
	}
	k_mutex_unlock(&ctx->iface_lock);
}

int modbus_notify_pmm_trans_finished(const int iface, int result)
{
	struct modbus_context *ctx;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return -EINVAL;
	}

	if (ctx->trans_status == MODBUS_TRANS_WAIT_NOTIFY) {
		k_timer_stop(&ctx->trans_timeout_timer);
		ctx->trans_status = MODBUS_TRANS_DONE;
		ctx->trans_result = result;
		LOG_DBG("Receive notification of transparent finished and submit work");
		k_work_submit(&ctx->server_work);
	}
	return 0;
}

struct modbus_pmm_id *modbus_find_pmm_id(const int iface, uint8_t id)
{
	struct modbus_context *ctx;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return NULL;
	}

	if (!ctx->pmm_server) {
		LOG_ERR("Interface %u not pmm server", iface);
		return NULL;
	}

	return _modbus_find_pmm_id(ctx, id);
}

int modbus_register_pmm_id(const int iface, uint8_t id, uint8_t trans_id)
{
	struct modbus_context *ctx;
	struct modbus_pmm_id *pmm_id;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return -EINVAL;
	}

	if (!ctx->pmm_server) {
		LOG_ERR("Interface %u not pmm server", iface);
		return -EINVAL;
	}

	if (_modbus_find_pmm_id(ctx, id)) {
		LOG_DBG("Id already exists");
		return -EINVAL;
	}

	pmm_id = k_malloc(sizeof(struct modbus_pmm_id));
	if (pmm_id == NULL)
		return -ENOMEM;

	pmm_id->id = id;
	pmm_id->trans = trans_id != TRANSPARENT_NOT_SUPPORT ? true : false;
	pmm_id->trans_id = trans_id;

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);
	sys_slist_append(&ctx->pmm_ids, &pmm_id->node);
	k_mutex_unlock(&ctx->iface_lock);

	return 0;
}

int modbus_unregister_pmm_id(const int iface, uint8_t id)
{
	struct modbus_context *ctx;
	struct modbus_pmm_id *pmm_id;
	bool found = false;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return -EINVAL;
	}

	if (!ctx->pmm_server) {
		LOG_ERR("Interface %u not pmm server", iface);
		return -EINVAL;
	}

	k_mutex_lock(&ctx->iface_lock, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&ctx->pmm_ids, pmm_id, node) {
		if (pmm_id->id == id) {
			sys_slist_remove(&ctx->pmm_ids, NULL, &pmm_id->node);
			found = true;
			break;
		}
	}
	k_mutex_unlock(&ctx->iface_lock);

	if (!found)
		return -EINVAL;

	k_free(pmm_id);

	return 0;
}

int modbus_unregister_all_pmm_id(const int iface)
{
	struct modbus_context *ctx;

	ctx = modbus_get_context(iface);
	if (ctx == NULL) {
		LOG_ERR("Interface %u not initialized", iface);
		return -EINVAL;
	}

	if (!ctx->pmm_server) {
		LOG_ERR("Interface %u not pmm server", iface);
		return -EINVAL;
	}

	_modbus_unregister_all_pmm_id(ctx);

	return 0;
}
/* liteon end */