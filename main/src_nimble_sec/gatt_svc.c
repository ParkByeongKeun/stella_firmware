/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gatt_svc.h"
#include "common.h"
#include "gap.h"
#include "heart_rate.h"
#include "led.h"

/* Private function declarations */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg);
static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);
/* Private variables */

//  //    6 #define SERVICE_UUID              "fb1e4001-54ae-4a28-9f74-dfccb248601d"
//  //    7 #define CHARACTERISTIC_UUID_RX    "fb1e4002-54ae-4a28-9f74-dfccb248601d"
//  //    8 #define CHARACTERISTIC_UUID_TX    "fb1e4003-54ae-4a28-9f74-dfccb248601d"

/* Heart rate service */
static const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x4001);
//  static const ble_uuid128_t heart_rate_svc_uuid = //BLE_UUID16_INIT(0x4001);
//  			 BLE_UUID128_INIT(0x1d, 0x60, 0x48, 0xb2, 0xcc, 0xdf,  0x74, 0x9f,  0x28, 0x4a,  0xae, 0x54,  0x01, 0x40, 0x1e, 0xfb);

static uint8_t heart_rate_chr_val[2] = {0};

//  static uint16_t heart_rate_chr_val_handle;
uint16_t heart_rate_chr_val_handle;

static const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x4002);
//  static const ble_uuid128_t heart_rate_chr_uuid = // BLE_UUID16_INIT(0x4002);
//  			 BLE_UUID128_INIT(0x1d, 0x60, 0x48, 0xb2, 0xcc, 0xdf,  0x74, 0x9f,  0x28, 0x4a,  0xae, 0x54,  0x03, 0x40, 0x1e, 0xfb);

uint16_t heart_rate_chr_conn_handle = 0;
static bool heart_rate_chr_conn_handle_inited = false;
static bool heart_rate_ind_status = false;

uint16_t conn_handle;
char notification[128];
extern bool notify_state;


extern SemaphoreHandle_t sema_ble_send_noti ;

//  static const ble_uuid16_t wearable_svc_uuid = BLE_UUID16_INIT(0x4001);
//  static const ble_uuid128_t wearable_uuid =
//  	BLE_UUID128_INIT(0x1d, 0x60, 0x48, 0xb2, 0xcc, 0xdf,  0x74, 0x9f,  0x28, 0x4a,  0xae, 0x54,  0x03, 0x40, 0x1e, 0xfb);

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &heart_rate_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {/* Heart rate characteristic */
              .uuid = &heart_rate_chr_uuid.u,
              .access_cb = heart_rate_chr_access,
//                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE |
//                         BLE_GATT_CHR_F_READ_ENC,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
              .val_handle = &heart_rate_chr_val_handle},
             {
                 0, /* No more characteristics in this service. */
             }}},


    {
        0, /* No more services. */
    },
};

void vTasksendNotification_for_keepalive() //! For sending notifications periodically as freetos task(after setting value of variable"notification")
{
  int rc;
  struct os_mbuf *om;
  static int val_increase = 0;
  while (1)
  {
    if (notify_state) //!! This value is checked so that we don't send notifications if no one has subscribed to our notification handle.
    {

	  xSemaphoreTake(sema_ble_send_noti, portMAX_DELAY);
      sprintf(notification, "KEEP_ALIVE,%d", val_increase++);
      om = ble_hs_mbuf_from_flat(notification, strlen(notification));
      ESP_LOGW("shcho", "notification(1)=%s", notification);

//        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
      rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
      printf("\n rc=%d\n", rc);

      if (rc != 0)
      {
        printf("\n error notifying; rc\n");
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
	  xSemaphoreGive(sema_ble_send_noti);


//  //        om = ble_hs_mbuf_from_flat(notification, sizeof(notification));
//  
//        sprintf(notification, "CO2,%d", 111+(val_increase++));
//        om = ble_hs_mbuf_from_flat(notification, strlen(notification));
//        ESP_LOGW("shcho", "notification(1)=%s", notification);
//  
//  //        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
//        rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
//        printf("\n rc=%d\n", rc);
//  
//        if (rc != 0)
//        {
//          printf("\n error notifying; rc\n");
//        }
//  
//  //        vTaskDelay(200 / portTICK_PERIOD_MS);
//        sprintf(notification, "Temperature,%.1f", 11.1+(float)(val_increase++));
//        om = ble_hs_mbuf_from_flat(notification, strlen(notification));
//        ESP_LOGW("shcho", "notification(2)=%s", notification);
//  
//  //        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
//        rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
//        printf("\n rc=%d\n", rc);
//  
//        if (rc != 0)
//        {
//          printf("\n error notifying; rc\n");
//        }
    }
    else
    {
      printf("No one subscribed to notifications\n");
//        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
//      vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}
int ble_send_noti_float(char *id, float value)
{
  	int rc = -1;
  	struct os_mbuf *om;
	  
    if (notify_state) 
	//!! This value is checked so that we don't send notifications 
	//if no one has subscribed to our notification handle.
	{
		xSemaphoreTake(sema_ble_send_noti, portMAX_DELAY);

		memset(notification, 0, sizeof(notification));
		sprintf(notification, "%s,%.4f", id, value);


		om = ble_hs_mbuf_from_flat(notification, strlen(notification));
		ESP_LOGW("shcho", "notification(1)=%s", notification);

//        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
		rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
		printf("\n rc=%d\n", rc);
		if (rc != 0)
		{
			printf("\n error notifying; rc(%s)\n", id);
		}

      	vTaskDelay(100 / portTICK_PERIOD_MS);
		xSemaphoreGive(sema_ble_send_noti);
	}
	return rc;
}


int ble_send_noti_int(char *id, int value)
{
  	int rc = -1;
  	struct os_mbuf *om;
	  
    if (notify_state) 
	//!! This value is checked so that we don't send notifications 
	//if no one has subscribed to our notification handle.
	{
		xSemaphoreTake(sema_ble_send_noti, portMAX_DELAY);

		memset(notification, 0, sizeof(notification));
		sprintf(notification, "%s,%d", id, value);

		om = ble_hs_mbuf_from_flat(notification, strlen(notification));
		ESP_LOGW("shcho", "notification(1)=%s", notification);

//        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
		rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
		printf("\n rc=%d\n", rc);
		if (rc != 0)
		{
			printf("\n error notifying; rc(%s)\n", id);
		}

      	vTaskDelay(100 / portTICK_PERIOD_MS);
		xSemaphoreGive(sema_ble_send_noti);
	}
	return rc;
}

int ble_send_noti_str(char *id, char* val_str)
{
  	int rc = -1;
  	struct os_mbuf *om;
	  
    if (notify_state) 
	//!! This value is checked so that we don't send notifications 
	//if no one has subscribed to our notification handle.
	{
		xSemaphoreTake(sema_ble_send_noti, portMAX_DELAY);

		memset(notification, 0, sizeof(notification));
		sprintf(notification, "%s,%s", id, val_str);

		om = ble_hs_mbuf_from_flat(notification, strlen(notification));
		ESP_LOGW("shcho", "notification(1)=%s", notification);

//        rc = ble_gattc_notify_custom(conn_handle, notification_handle, om);
		rc = ble_gattc_notify_custom(conn_handle, heart_rate_chr_val_handle, om);
		printf("\n rc=%d\n", rc);
		if (rc != 0)
		{
			printf("\n error notifying; rc(%s)\n", id);
		}
		xSemaphoreGive(sema_ble_send_noti);
	}
	return rc;
}



/* Private functions */
static int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
    /* Local variables */
    int rc = 0 ;

    /* Handle access events */
    /* Note: Heart rate characteristic is read only */
    switch (ctxt->op) {

    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR: //Indication은 : Phone에서 계속 읽어야 한다.
        /* Verify connection handle */
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d",
                     conn_handle, attr_handle);
        } else {
            ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d",
                     attr_handle);
        }

        /* Verify attribute handle */
        if (attr_handle == heart_rate_chr_val_handle) {
			//=====================================================
//              /* Update access buffer value */
//              heart_rate_chr_val[1] = get_heart_rate();
//              rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val,
//                                  sizeof(heart_rate_chr_val));
			//-----------------------------------------------------
			char buffer[256] ;
			memset( buffer, 0, sizeof(buffer));
			sprintf(buffer, "CO2,%d", get_heart_rate());
			ESP_LOGW("shcho_debug", "%s", buffer);
			//              sprintf(buffer, "S_0_4,%d", get_heart_rate());
			rc = os_mbuf_append(ctxt->om, buffer, strlen(buffer)+1);
			//=====================================================
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        goto error;

    /* Unknown event */
    default:
        goto error;
    }

error:
    ESP_LOGE(
        TAG,
        "unexpected access operation to heart rate characteristic, opcode: %d",
        ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

//  static int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
//                            struct ble_gatt_access_ctxt *ctxt, void *arg) {
//      /* Local variables */
//      int rc;
//  
//      /* Handle access events */
//      /* Note: LED characteristic is write only */
//      switch (ctxt->op) {
//  
//      /* Write characteristic event */
//      case BLE_GATT_ACCESS_OP_WRITE_CHR:
//          /* Verify connection handle */
//          if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
//              ESP_LOGI(TAG, "characteristic write; conn_handle=%d attr_handle=%d",
//                       conn_handle, attr_handle);
//          } else {
//              ESP_LOGI(TAG,
//                       "characteristic write by nimble stack; attr_handle=%d",
//                       attr_handle);
//          }
//  
//          /* Verify attribute handle */
//          if (attr_handle == led_chr_val_handle) {
//              /* Verify access buffer length */
//              if (ctxt->om->om_len == 1) {
//                  /* Turn the LED on or off according to the operation bit */
//                  if (ctxt->om->om_data[0]) {
//                      led_on();
//                      ESP_LOGI(TAG, "led turned on!");
//                  } else {
//                      led_off();
//                      ESP_LOGI(TAG, "led turned off!");
//                  }
//              } else {
//                  goto error;
//              }
//              return rc;
//          }
//          goto error;
//  
//      /* Unknown event */
//      default:
//          goto error;
//      }
//  
//  error:
//      ESP_LOGE(TAG,
//               "unexpected access operation to led characteristic, opcode: %d",
//               ctxt->op);
//      return BLE_ATT_ERR_UNLIKELY;
//  }

/* Public functions */
// shcho : for BLE Non Security
void send_heart_rate_indication(void) {
    if (heart_rate_ind_status && heart_rate_chr_conn_handle_inited) {
        ble_gatts_indicate(heart_rate_chr_conn_handle,
                           heart_rate_chr_val_handle);
        ESP_LOGI(TAG, "shcho :계속 보냄, heart rate indication sent!");
    }
}
//  //shcho  : for BLE Security
//  void send_heart_rate_indication(void) {
//      /* Check if connection handle is initialized */
//      if (!heart_rate_chr_conn_handle_inited) {
//          return;
//      }
//  
//      /* Check indication and security status */
//      if (heart_rate_ind_status &&
//          is_connection_encrypted(heart_rate_chr_conn_handle)) {
//          ble_gatts_indicate(heart_rate_chr_conn_handle,
//                             heart_rate_chr_val_handle);
//      }
//  }

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op) {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        assert(0);
        break;
    }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

//  shcho: for BLE Non Security
void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
    /* Check connection handle */
    if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle);
    } else {
        ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d",
                 event->subscribe.attr_handle);
    }

    /* Check attribute handle */
    if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
        /* Update heart rate subscription status */
        heart_rate_chr_conn_handle = event->subscribe.conn_handle;
        heart_rate_chr_conn_handle_inited = true;
        heart_rate_ind_status = event->subscribe.cur_indicate;
    }
}
//  shcho: for BLE Security
//  int gatt_svr_subscribe_cb(struct ble_gap_event *event) {
//      /* Check attribute handle */
//      if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
//          /* Update heart rate subscription status */
//          heart_rate_chr_conn_handle = event->subscribe.conn_handle;
//          heart_rate_chr_conn_handle_inited = true;
//          heart_rate_ind_status = event->subscribe.cur_indicate;
//  
//          /* Check security status */
//          if (!is_connection_encrypted(event->subscribe.conn_handle)) {
//              ESP_LOGE(TAG, "failed to subscribe to heart rate measurement, "
//                            "connection not encrypted!");
//              return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
//          }
//      }
//      return 0;
//  }

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
    /* Local variables */
    int rc;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
