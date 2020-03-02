/*
 * main.c
 *
 *  Created on: 2 mars 2020
 *      Author: arion
 */

#include <stdio.h>

#include "emulator.h"
#include "RocketFS/rocket_fs.h"

void debug(const char* message) {
	printf("%s", message);
}

void handle_telemetry(uint32_t timestamp, float baro_altitude, float speed, float temperature, float pressure, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ);
void handle_gps(uint32_t timestamp, float hdop, float latitude, float longitude, float altitude, uint8_t sats);
void handle_state(uint32_t timestamp, uint8_t state);

FILE* telemetry_file;
FILE* gps_file;
FILE* state_file;

int main() {
	FILE* log = fopen("FLASH.DMP", "rb");

	telemetry_file = fopen("AV TELEMETRY.log", "w");
	gps_file = fopen("AV GPS.log", "w");
	state_file = fopen("AV STATUS.log", "w");

	if(log)

	emu_init(log);

	FileSystem fs = { 0 };
	rocket_fs_debug(&fs, &debug);
	rocket_fs_device(&fs, "emulator", FS_ADDRESSABLE_SPACE, FS_SUBSECTOR_SIZE);
	rocket_fs_bind(&fs, &emu_read, &emu_write, &emu_erase_subsector);
	rocket_fs_mount(&fs);

	File* flight = rocket_fs_getfile(&fs, "FLIGHT");

	Stream stream;
	rocket_fs_stream(&stream, &fs, flight, OVERWRITE);


	float latitude;
	float longitude;
	float altitude;
	float hdop;
	uint8_t sats;

	float baro_altitude;
	float speed;
	float temperature;
	float base_pressure;
	float pressure;
	float accelX;
	float accelY;
	float accelZ;
	float gyroX;
	float gyroY;
	float gyroZ;


	while(1) {
		uint8_t buffer[4];
		stream.read(buffer, 4);
		int32_t data = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
		uint8_t id = stream.read8();
		stream.read(buffer, 3);
		uint32_t timestamp = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

		switch(id) {
		case 0:
			pressure = (float) data / 1e2f;
			handle_telemetry(timestamp, baro_altitude, speed, temperature, pressure, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
			break;
		case 1:
			accelX = (float) data;
			break;
		case 2:
			accelY = (float) data;
			break;
		case 3:
			accelZ = (float) data;
			handle_telemetry(timestamp, baro_altitude, speed, temperature, pressure, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
			break;
		case 4:
			gyroX = (float) data;
			break;
		case 5:
			gyroY = (float) data;
			break;
		case 6:
			gyroZ = (float) data;
			break;
		case 7:
			hdop = (float) data / 1e3f;
			break;
		case 8:
			latitude = (float) data / 1e6f;
			break;
		case 9:
			longitude = (float) data / 1e6f;
			break;
		case 10:
			altitude = (float) data;
			break;
		case 11:
			sats = data;
			handle_gps(timestamp, latitude, longitude, altitude, hdop, sats);
			break;
		case 12:
			temperature = (float) data / 1e2f;
			break;
		case 13:
			base_pressure = (float) data / 1e2f;
			break;
		case 38:
			break;
		case 42:
			baro_altitude = (float) data;
			break;
		case 45:
			speed = (float) data;
			break;
		case 50:
			handle_state(timestamp, data);
			break;
		case 255:
			break;
		default:
			printf("Unhandled CAN message: %d\n", id);
		}
	}


	fclose(log);
	fclose(telemetry_file);
	fclose(gps_file);
	fclose(state_file);

	return 0;
}

uint32_t sensor_packet_id = 0;
uint32_t gps_packet_id = 0;

void handle_telemetry(uint32_t timestamp, float baro_altitude, float speed, float temperature, float pressure, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
	printf("3\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", timestamp, sensor_packet_id, baro_altitude, speed, temperature, pressure, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
	sensor_packet_id++;
}

void handle_gps(uint32_t timestamp, float hdop, float latitude, float longitude, float altitude, uint8_t sats) {
	printf("1\t%d\t%d\tGPS\t%f\t%f\t%f\t%f\t%f\t%d\n", timestamp, gps_packet_id, hdop, altitude, latitude, longitude, altitude, sats);
	gps_packet_id++;
}

void handle_state(uint32_t timestamp, uint8_t state) {
	switch(state) {
	case 0:
		printf("2\t%d\t%d\tAVState SLEEP\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 1:
		printf("2\t%d\t%d\tAVState CALIBRATION\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 2:
		printf("2\t%d\t%d\tAVState IDLE\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 3:
		printf("2\t%d\t%d\tAVState FILLING\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 4:
		printf("2\t%d\t%d\tAVState LIFTOFF\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 5:
		printf("2\t%d\t%d\tAVState COAST\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 6:
		printf("2\t%d\t%d\tAVState PRIMARY EVENT\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 7:
		printf("2\t%d\t%d\tAVState SECONDARY EVENT\t42.000000\n", timestamp, sensor_packet_id);
		break;
	case 8:
		printf("2\t%d\t%d\tAVState TOUCH DOWN\t42.000000\n", timestamp, sensor_packet_id);
		break;
	}

	sensor_packet_id++;
}
