/*!
- MindPlus
- esp32s3bit
 *
 */
#include <DFRobot_Iot.h>
#include "unihiker_k10.h"
#include "unihiker_k10_webcam.h"

// HTTP Server 新增
#include "esp_http_server.h"

// 函数声明
void DF_ZuoCeDianJi(float mind_n_speed, float mind_n_direction);
void DF_common_move(float mind_n_speed, float mind_n_direction, float mind_n_speed_gap);
void DF_YouCeDianJi(float mind_n_speed, float mind_n_direction);
void DF_DianJiShiNen();
void DF_XiaoCheTingZhi();

// HTTP Server 新增
bool enableControlHttpServer();

// 创建对象
UNIHIKER_K10               k10;
uint8_t                    screen_dir=2;
DFRobot_Iot                myIot;
unihiker_k10_webcam webcam ;

// HTTP Server 新增
static httpd_handle_t control_httpd = NULL;


// 主程序开始
void setup() {
	k10.begin();
	Serial.begin(9600);
	k10.initScreen(screen_dir);
	k10.initBgCamerImage();
	k10.setBgCamerImage(false);
	k10.creatCanvas();
	k10.setBgCamerImage(true);
	myIot.wifiConnect("lkj", "00000000");
	while (!myIot.wifiStatus()) {}
	Serial.println("WIFI连接成功");
	webcam.enableWebcam();
	DF_DianJiShiNen();

	// HTTP Server 新增
	enableControlHttpServer();
}

void loop() {
	Serial.println(myIot.getWiFiLocalIP());
}


// 自定义函数
void DF_ZuoCeDianJi(float mind_n_speed, float mind_n_direction) {
	if ((mind_n_direction==1)) {
		digital_write(eP2, HIGH);
		digital_write(eP3, LOW);
	}
	else {
		digital_write(eP2, LOW);
		digital_write(eP3, HIGH);
	}
	analogWrite(P0, map(mind_n_speed, 0, 1023, 0, 255));
}

void DF_common_move(float mind_n_speed, float mind_n_direction, float mind_n_speed_gap) {
	DF_ZuoCeDianJi((mind_n_speed - mind_n_speed_gap), mind_n_direction);
	DF_YouCeDianJi(mind_n_speed, mind_n_direction);
}

void DF_YouCeDianJi(float mind_n_speed, float mind_n_direction) {
	if ((mind_n_direction==1)) {
		digital_write(eP4, HIGH);
		digital_write(eP8, LOW);
	}
	else {
		digital_write(eP4, LOW);
		digital_write(eP8, HIGH);
	}
	analogWrite(P1, map(mind_n_speed, 0, 1023, 0, 255));
}

void DF_DianJiShiNen() {
	digital_write(eP6, HIGH);
}

void DF_XiaoCheTingZhi() {
	analogWrite(P0, map(0, 0, 1023, 0, 255));
	analogWrite(P1, map(0, 0, 1023, 0, 255));
}


// ==================================================
// 以下全部为 HTTP Server 新增功能
// ==================================================

esp_err_t control_http_handler(httpd_req_t *req) {
	size_t query_len = httpd_req_get_url_query_len(req);

	if (query_len == 0 || query_len >= 64) {
		return httpd_resp_send_err(
			req,
			HTTPD_400_BAD_REQUEST,
			"use /control?gap=100"
		);
	}

	char query[64];
	char gap_text[16];

	if (
		httpd_req_get_url_query_str(
			req,
			query,
			sizeof(query)
		) != ESP_OK
	) {
		return httpd_resp_send_err(
			req,
			HTTPD_400_BAD_REQUEST,
			"invalid query"
		);
	}

	if (
		httpd_query_key_value(
			query,
			"gap",
			gap_text,
			sizeof(gap_text)
		) != ESP_OK
	) {
		return httpd_resp_send_err(
			req,
			HTTPD_400_BAD_REQUEST,
			"missing gap"
		);
	}

	/*
	 * 与原 MQTT 回调保持相同调用方式：
	 *
	 * DF_common_move(1000, 1, message.toInt());
	 */
	int speed_gap = String(gap_text).toInt();

	DF_common_move(
		1000,
		1,
		speed_gap
	);

	char response[64];

	snprintf(
		response,
		sizeof(response),
		"{\"ok\":true,\"gap\":%d}",
		speed_gap
	);

	httpd_resp_set_type(
		req,
		"application/json"
	);

	httpd_resp_set_hdr(
		req,
		"Access-Control-Allow-Origin",
		"*"
	);

	return httpd_resp_sendstr(
		req,
		response
	);
}


esp_err_t stop_http_handler(httpd_req_t *req) {
	DF_XiaoCheTingZhi();

	httpd_resp_set_type(
		req,
		"application/json"
	);

	httpd_resp_set_hdr(
		req,
		"Access-Control-Allow-Origin",
		"*"
	);

	return httpd_resp_sendstr(
		req,
		"{\"ok\":true,\"stopped\":true}"
	);
}


bool enableControlHttpServer() {
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();

	/*
	 * 摄像头 Server 使用：
	 *   HTTP 端口 80
	 *   控制端口 32768
	 *
	 * 新 Server 使用不同端口，防止冲突。
	 */
	config.server_port = 81;
	config.ctrl_port = 32769;
	config.max_uri_handlers = 2;


	httpd_uri_t control_uri = {};

	control_uri.uri = "/control";
	control_uri.method = HTTP_GET;
	control_uri.handler = control_http_handler;
	control_uri.user_ctx = NULL;


	httpd_uri_t stop_uri = {};

	stop_uri.uri = "/stop";
	stop_uri.method = HTTP_GET;
	stop_uri.handler = stop_http_handler;
	stop_uri.user_ctx = NULL;


	if (
		httpd_start(
			&control_httpd,
			&config
		) != ESP_OK
	) {
		Serial.println("HTTP控制服务启动失败");

		control_httpd = NULL;

		return false;
	}


	if (
		httpd_register_uri_handler(
			control_httpd,
			&control_uri
		) != ESP_OK
		||
		httpd_register_uri_handler(
			control_httpd,
			&stop_uri
		) != ESP_OK
	) {
		Serial.println("HTTP控制接口注册失败");

		httpd_stop(control_httpd);
		control_httpd = NULL;

		return false;
	}


	Serial.println("HTTP控制服务启动成功，端口81");

	return true;
}