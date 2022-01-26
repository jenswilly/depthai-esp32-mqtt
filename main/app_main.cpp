/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "decode_raw_mobilenet.h"
#include "esp32_spi_impl.h"
#include "spi_api.hpp"

#include <string>
#include <vector>
#include <chrono>

#include "rect.hpp"

// JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;

static const char *TAG = "MQTT_EXAMPLE";

/// Prefix used as the MQTT root element
#define MQTT_PREFIX "esp32-depthai"

/// MQTT topics
static const std::string mqtt_ping_topic = MQTT_PREFIX "/ping/" CONFIG_BROKER_CLIENTID;
static const std::string mqtt_detections_topic = MQTT_PREFIX "/detections/" CONFIG_BROKER_CLIENTID;

/// Global MQTT client handle
esp_mqtt_client_handle_t client;

/// Timestamp for throttling
auto lastTimestamp = std::chrono::steady_clock::now();
#define THROTTLE_INTERVAL 5

/// Minimum confidence for detection to be accepted
#define MIN_CONFIDENCE 0.8

#define MAX_DETECTIONS 16

static const char* METASTREAM = "spimetaout";
static const char* PREVIEWSTREAM = "spipreview";

extern "C" {
   void app_main();
}

/// Category labels
static std::vector<std::string> labels = {
	"background",
	"aeroplane",
	"bicycle",
	"bird",
	"boat",
	"bottle",
	"bus",
	"car",
	"cat",
	"chair",
	"cow",
	"diningtable",
	"dog",
	"horse",
	"motorbike",
	"person",
	"pottedplant",
	"sheep",
	"sofa",
	"train",
	"tvmonitor"
};

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/**
 * @brief Hardcoded "detections slots"
 * 
 * The values are Rect structs and the coordinates are from 0 - 1 on both axes. * 
 */
static std::vector<Rect> slots = {
    {0, 0, 0.5, 1}, // Left half
    {0.5, 0, 1, 1}  // Right half
};

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_t*)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

        // mqtt_ping_topic += "/ping/";
        // mqtt_ping_topic += CONFIG_BROKER_CLIENTID;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Send MQTT ping including client ID on startup
        msg_id = esp_mqtt_client_publish(client, mqtt_ping_topic.c_str(), "true", 4, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
        ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .port = CONFIG_BROKER_PORT,
        .client_id = CONFIG_BROKER_CLIENTID,
        .username = CONFIG_BROKER_USERNAME,
        .password = CONFIG_BROKER_PASSWORD,
    };

    ESP_LOGI(TAG, "[MQTT] host: %s, port: %d, username: %s, password: ******, client: %s", 
        mqtt_cfg.uri,
        mqtt_cfg.port,
        mqtt_cfg.username,
        mqtt_cfg.client_id );

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void run_demo() {
	uint8_t req_success = 0;

	dai::SpiApi mySpiApi;
	mySpiApi.set_send_spi_impl(&esp32_send_spi);
	mySpiApi.set_recv_spi_impl(&esp32_recv_spi);

	while(1) {
		// ----------------------------------------
		// basic example of receiving data and metadata from messages.
		// ----------------------------------------
		dai::Message received_msg;

		if(mySpiApi.req_message(&received_msg, METASTREAM)) {
			// Parse metadata
			dai::RawImgDetections det;
			mySpiApi.parse_metadata(&received_msg.raw_meta, det);

            // Throttle to avoid flooding the MQTT broker
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = now - lastTimestamp;

			if(det.detections.size() > 0 && elapsed.count() >= THROTTLE_INTERVAL ) {
                lastTimestamp = now;

                // Create root object
                json jsonRoot;
                jsonRoot["detections"] = json::array();

                // Iterate detections
				for(const dai::ImgDetection& det : det.detections) {
                    // Ignore if not specific labels (person, car, cat)
                    if(det.label != 7 && det.label != 8 && det.label != 15) {   // TODO: change to constant vector and use find()
                        printf("Ignoring detection with label %d\n", det.label);
                        continue;
                    }
                    if(det.confidence <= MIN_CONFIDENCE) {
                        // Too low confidence: ignore
                        continue;
                    }

                    // JSON output
                    json detection = {
                        {"label", det.label},
                        {"confidence", det.confidence},
                        {"rect", {
                            {"xmin", det.xmin},
                            {"ymin", det.ymin},
                            {"xmax", det.xmax},
                            {"ymax", det.ymax},
                        }},
                        {"overlaps", json::array()}
                    };

                    // Add slots overlaps
                    Rect detectionRect = {det.xmin, det.ymin, det.xmax, det.ymax};

                    for( auto slot: slots ) {
                        detection["overlaps"].push_back(slot.overlapRatio(&detectionRect));
                    }
                    
                    // Add category name or NULL if index out-of-bounds
                    detection["category"] = (det.label <= labels.size() ? labels[det.label] : NULL);

                    jsonRoot["detections"].push_back(detection);
				}

                // Only send if we found any relevant categories
                if(jsonRoot["detections"].size() > 0) {
                    std::string jsonString = jsonRoot.dump();
                    if(!CONFIG_DEBUG_MODE)
                        // Only send if not running in debug mode
                        esp_mqtt_client_publish(client, mqtt_detections_topic.c_str(), jsonString.c_str(), jsonString.length(), 0, 0);
                    else
                        // Debug mode from config: only print JSON
                        ESP_LOGI(TAG, "MQTT (unsent):\n%s", jsonString.c_str());
                }
			}

			// free up resources once you're done with the message.
			// and pop the message from the queue
			mySpiApi.free_message(&received_msg);
			mySpiApi.spi_pop_messages();
		}
	}
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Connect to MQTT broker
    mqtt_app_start();
    ESP_LOGI(TAG, "MQTT client started...");

    // Init spi for the esp32 and send pin
    init_esp32_spi();
    ESP_LOGI(TAG, "SPI initialized.");

    // Run SPI listener loop
    run_demo(); // Never completes
}
