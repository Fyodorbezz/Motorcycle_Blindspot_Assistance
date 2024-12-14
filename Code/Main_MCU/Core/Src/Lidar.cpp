#include "main.h"

extern CORDIC_HandleTypeDef hcordic;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern USART_HandleTypeDef husart6;
extern DMA_NodeTypeDef Node_GPDMA1_Channel0;
extern DMA_QListTypeDef List_GPDMA1_Channel0;
extern DMA_HandleTypeDef handle_GPDMA1_Channel0;

extern RTC_HandleTypeDef hrtc;

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

double radians(double angle){
	return (angle*PI)/180;
}

GPIO_TypeDef *connect_det_ports[4] = {CONNECT_DET_1_GPIO_Port, CONNECT_DET_2_GPIO_Port, CONNECT_DET_3_GPIO_Port, CONNECT_DET_4_GPIO_Port};
uint16_t connect_det_pins[4] = {CONNECT_DET_1_Pin, CONNECT_DET_2_Pin, CONNECT_DET_3_Pin, CONNECT_DET_4_Pin};

GPIO_TypeDef *connect_led_ports[4] = {CONNECT_LED_1_GPIO_Port, CONNECT_LED_2_GPIO_Port, CONNECT_LED_3_GPIO_Port, CONNECT_LED_4_GPIO_Port};
uint16_t connect_led_pins[4] = {CONNECT_LED_1_Pin, CONNECT_LED_2_Pin, CONNECT_LED_3_Pin, CONNECT_LED_4_Pin};

GPIO_TypeDef *working_led_ports[4] = {WORKING_LED_1_GPIO_Port, WORKING_LED_2_GPIO_Port, WORKING_LED_3_GPIO_Port, WORKING_LED_4_GPIO_Port};
uint16_t working_led_pins[4] = {WORKING_LED_1_Pin, WORKING_LED_2_Pin, WORKING_LED_3_Pin, WORKING_LED_4_Pin};

uint16_t timer_channels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

struct Lidar_header{
  uint8_t length_MSB;
  uint8_t length_LSB;
  uint8_t protocol;
  uint8_t type;
  uint8_t command;
  uint8_t payload_length_MSB;
  uint8_t payload_length_LSB;
};

struct Points_header{
  uint8_t RPM;
  uint8_t offset_angle_MSB;
  uint8_t offset_angle_LSB;
  uint8_t start_angle_MSB;
  uint8_t start_angle_LSB;
};

struct Lidar_point_message{
  uint8_t signal_quality;
  uint8_t distance_MSB;
  uint8_t distance_LSB;
};

struct Lidar_point{
  double angle;
  double distance;
};

struct Lidar_point_processed{
  double angle = 0;
  double distance = 0;
  double x = 0;
  double y = 0;
};

class Lidar_functions{
	public:
	uint8_t lidar_index = 0;

	uint8_t RxData[BUFFER_LENGTH];
	uint16_t current_index = 0;
	uint8_t packet_recieved = 0;
	bool first_packet = 0;

	Lidar_header header;
	Points_header points_header;
	Lidar_point_message lidar_point_message;

	Lidar_point lidar_points[400] = {};
	uint16_t current_point = 0;
	uint16_t packets_count = 0;
	bool new_rotation = 0;

	Lidar_point_processed lidar_points_processed[400] = {};
	Lidar_point_processed lidar_points_processed_old[400] = {};
	uint16_t points_amount = 0;
	uint16_t points_amount_old = 0;
	int16_t y_shift = 0;
	int8_t invert = 0;

	double lidar_RPS = 0;
	uint8_t status = 0;
	uint8_t connected = 0;
	uint32_t last_packet_recieved = 0;
	uint8_t last_connected = 0;
	uint8_t normal_working = 0;
	uint32_t last_conversion = 0;
	uint8_t disconnected = 0;

	void process_rotation(){
		memcpy(lidar_points_processed_old, lidar_points_processed, points_amount);
		points_amount_old = points_amount;
		points_amount = 0;

		for (int i=0; i<current_point; i++){
			if (lidar_points[i].distance > 50 && lidar_points[i].distance < 2000){
				lidar_points_processed[points_amount].angle = lidar_points[i].angle;
				lidar_points_processed[points_amount].distance = lidar_points[i].distance;
				lidar_points_processed[points_amount].x = -(lidar_points_processed[points_amount].distance - 20) * cos(radians(lidar_points_processed[points_amount].angle)) * invert;
				lidar_points_processed[points_amount].y = (lidar_points_processed[points_amount].distance - 20) * sin(radians(lidar_points_processed[points_amount].angle)) * invert + y_shift;
				points_amount++;
			}
		}
		current_point = 0;
		new_rotation = 1;

	}

	void get_packet_from_lidar(){
		last_packet_recieved = HAL_GetTick();
		if (first_packet == 0){
			memset(RxData, '\0', BUFFER_LENGTH);
			first_packet = 1;
			return;
		}


		int add=0;
		while(RxData[(current_index + add)%BUFFER_LENGTH] != 0xAA && add <=BUFFER_LENGTH){
			add++;
		}

		//printf("Add %d", add);

		memcpy(&header, RxData+(current_index + add +1)%BUFFER_LENGTH, sizeof(header));
		if (header.command == NORMAL_WORKING){
			status = 1;
			memcpy(&points_header, RxData+(current_index + add + 1 + sizeof(header))%BUFFER_LENGTH, sizeof(points_header));

			lidar_RPS = points_header.RPM*0.05;

			double start_angle = (points_header.start_angle_MSB*256 + points_header.start_angle_LSB)*0.01;
			uint16_t points_in_message = (header.payload_length_LSB - 4)/3;
			double current_angle = 0;

			//printf("Ang %f", start_angle);

			for (int i=0; i < points_in_message; i++){
				memcpy(&lidar_point_message, RxData+(current_index + add + 1 + sizeof(header) + sizeof(points_header) + sizeof(lidar_point_message)*i)%BUFFER_LENGTH, sizeof(lidar_point_message));
				current_angle = start_angle + (24.0/points_in_message)*i;

				lidar_points[current_point%400].angle = current_angle;
				lidar_points[current_point%400].distance = (lidar_point_message.distance_MSB*256 + lidar_point_message.distance_LSB)*0.25;

				current_point ++;
			}

			packets_count ++;
		}
		else if(header.command == WRONG_SPEED){
			status = 0;
			lidar_RPS = RxData[(current_index + add + 1 + sizeof(header))%BUFFER_LENGTH]*0.05;
		}

		current_index += add + header.length_MSB*256 + header.length_LSB;
		current_index = current_index % BUFFER_LENGTH;
		//printf("\n");
		return;
	}

	void tick(){
		if (packet_recieved > 0){
			//get_packet_from_lidar();
			//printf(" %d ", HAL_GetTick());
			//printf("  %f \n", lidar_RPS);
			//printf("%d \n", packets_count);
			if (status == 0){
				//printf("Error %f \n", lidar_RPS);
			}
			//packet_recieved --;
		}
		if (packets_count >= 15){
			//printf("%d ", current_point);
			process_rotation();
			/*printf("%d ", lidar_index);
			printf("%d ", HAL_GetTick() - last_conversion);
			printf("%d ", points_amount);
			printf("%d ", packet_recieved);
			printf("%f \n", lidar_RPS);*/

			last_conversion = HAL_GetTick();
			packets_count = 0;
		}
	}

	void reset_lidar(){
		memset(RxData, '\0', BUFFER_LENGTH);
		current_index = 0;
		packet_recieved = 0;
		first_packet = 0;
		current_point = 0;
		connected = 1;
	}

	void update_connection_status(){
		if (HAL_GPIO_ReadPin(connect_det_ports[lidar_index - 1], connect_det_pins[lidar_index - 1]) == 0 && last_connected == 0 && connected == 0){
			HAL_GPIO_WritePin(connect_led_ports[lidar_index - 1], connect_led_pins[lidar_index - 1], GPIO_PIN_SET);
			reset_lidar();
			
			if (disconnected){
				__HAL_TIM_SET_COMPARE(&htim4, timer_channels[0], 0);
				__HAL_TIM_SET_COMPARE(&htim4, timer_channels[1], 0);
				__HAL_TIM_SET_COMPARE(&htim4, timer_channels[2], 0);
				__HAL_TIM_SET_COMPARE(&htim4, timer_channels[3], 0);
				HAL_Delay(1000);
				__NVIC_SystemReset();
			}
			
			__HAL_TIM_SET_COMPARE(&htim4, timer_channels[lidar_index - 1], LIDAR_SPEED);
		}
		else if(HAL_GPIO_ReadPin(connect_det_ports[lidar_index - 1], connect_det_pins[lidar_index - 1]) == 1 && last_connected == 1 && connected == 1){
			HAL_GPIO_WritePin(connect_led_ports[lidar_index - 1], connect_led_pins[lidar_index - 1], GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim4, timer_channels[lidar_index - 1], 0);
			
			disconnected = 1;
			connected = 0;
		}

		last_connected = HAL_GPIO_ReadPin(connect_det_ports[lidar_index - 1], connect_det_pins[lidar_index - 1]);

		if(status == 1 && connected == 1 && HAL_GetTick() - last_packet_recieved < 200){
			HAL_GPIO_WritePin(working_led_ports[lidar_index - 1], working_led_pins[lidar_index - 1], GPIO_PIN_SET);
			normal_working = 1;
		}
		else{
			HAL_GPIO_WritePin(working_led_ports[lidar_index - 1], working_led_pins[lidar_index - 1], GPIO_PIN_RESET);
			normal_working = 0;
		}
	}

};

Lidar_functions lidars[4];

struct Settings{
  uint8_t power;
  uint8_t range;
  uint8_t sensetivity;
  uint8_t collision;
  uint8_t autoBreak;
  uint8_t brightness_1;
  uint8_t brightness_2;
};
Settings settings;
uint8_t settings_rx_buffer[sizeof(settings)];

uint8_t new_settings = 0;

uint32_t first_reg, second_reg;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2){
		if(Size != BUFFER_LENGTH && Size != BUFFER_LENGTH/2){
			lidars[0].get_packet_from_lidar();
		}
	}
	  else if (huart->Instance == USART3){
			if(Size != BUFFER_LENGTH && Size != BUFFER_LENGTH/2){
				lidars[1].get_packet_from_lidar();
			}
		}
	  else if (huart->Instance == UART4){
			if(Size != BUFFER_LENGTH && Size != BUFFER_LENGTH/2){
				lidars[2].get_packet_from_lidar();
			}
		}
	  else if (huart->Instance == UART5){
			if(Size != BUFFER_LENGTH && Size != BUFFER_LENGTH/2){
				lidars[3].get_packet_from_lidar();
			}
		}

	  else if (huart->Instance == UART7 && HAL_GetTick()>1000){
			memcpy(&settings, settings_rx_buffer, sizeof(settings_rx_buffer));
			new_settings = 1;
			HAL_UARTEx_ReceiveToIdle_IT(&huart7, settings_rx_buffer, sizeof(settings_rx_buffer));
		}
	//printf("%d ", Size);
	//printf("%d ", huart->Instance == USART2);
	//printf("%d ", huart->Instance == USART3);
	//printf("%d ", huart->Instance == UART4);
	//printf("%d \n", huart->Instance == UART5);
}

uint32_t connect_update_timer = 0;
uint32_t main_calculation_timer = 0;

struct Geometry_point{
	int32_t x;
	int32_t y;
};

struct settings_profile{
	Geometry_point blind_spot_bounding_rects[5][2];
	Geometry_point blind_spot_zones_rects[5][2][2];
	Geometry_point dadnger_zones_bounding_rects[2][2];
	Geometry_point danger_zones_vertices[2][4];
};

double size_treshold = 8;

//double size_treshold = 3.5;

//double size_treshold = 7.5;

double collision_detection_time = 2000;


Geometry_point assembled_points[1200] = {};
uint16_t assembled_points_amount = 0;

class Zone{
	public:
	Geometry_point bounding_rect[2];

	int16_t point_indexes[300];
	uint16_t number_of_points = 0;

	Geometry_point zone_rects[2][2];

	int16_t subzone_points_indexes[3][300];
	uint16_t subzone_number_of_points[3] = {0, 0, 0};

	long double center_x = 0;
	long double center_y = 0;
	long double size_x = 0;
	long double size_y = 0;

	uint8_t object_zone = 0;

	void calculate_geomety(){
		object_zone = 0;
		for (int i=2; i>=0; i--){
			center_x = 0;
			center_y = 0;
			size_x = 0;
	 		size_y = 0;
			if (subzone_number_of_points[i] > POINTS_THRESHOLD){
				//printf("%d \n", subzone_number_of_points[i]);
				for (int j=0; j<subzone_number_of_points[i]; j++){
					center_x += assembled_points[subzone_points_indexes[i][j]].x;
					center_y += assembled_points[subzone_points_indexes[i][j]].y;
				}

				center_x = center_x / subzone_number_of_points[i];
				center_y = center_y / subzone_number_of_points[i];

				for (int j=0; j<subzone_number_of_points[i]; j++){
					size_x += pow(double(abs(center_x - assembled_points[subzone_points_indexes[i][j]].x)), 0.5);
					size_y += pow(double(abs(center_y - assembled_points[subzone_points_indexes[i][j]].y)), 0.5);
				}

				size_x = pow(size_x / subzone_number_of_points[i], 2);
				size_y = pow(size_y / subzone_number_of_points[i], 2);

				if ((size_x + size_y) / 2.0 > size_treshold){
					object_zone = i+1;
					return;
				}
			}
		}
	}

	void simple_mode_sort(){
		for(int i=0; i<3; i++){
			subzone_number_of_points[i] = 0;
		}

		for(int i = 0; i<number_of_points; i++){
			subzone_points_indexes[0][i] = point_indexes[i];
			subzone_number_of_points[0] ++;

			for (int j = 0; j<2; j++){
				if (assembled_points[point_indexes[i]].x > zone_rects[j][0].x && assembled_points[point_indexes[i]].x < zone_rects[j][1].x &&
					assembled_points[point_indexes[i]].y < zone_rects[j][0].y && assembled_points[point_indexes[i]].y > zone_rects[j][1].y){
					subzone_points_indexes[j+1][subzone_number_of_points[j+1]] = point_indexes[i];
					subzone_number_of_points[j+1] ++;
				}
			}
		}
	}

	void analyse_points(){
		number_of_points = 0;

		for(int i=0; i<assembled_points_amount; i++){
			if(assembled_points[i].x > bounding_rect[0].x && assembled_points[i].x < bounding_rect[1].x && assembled_points[i].y < bounding_rect[0].y && assembled_points[i].y > bounding_rect[1].y){
				point_indexes[number_of_points] = i;
				number_of_points ++;
			}
		}

		simple_mode_sort();

		//printf("%d %d %d %d %d\n", assembled_points_amount, number_of_points,  subzone_number_of_points[0], subzone_number_of_points[1], subzone_number_of_points[2]);
		calculate_geomety();

		//printf("%d %f %f %f %f %d\n", HAL_GetTick() - main_calculation_timer, center_x, center_y, size_x, size_y, object_zone);
	}
	
};

class Comnplicated_zone{
	public:
	Geometry_point bounding_rect[2];
	Geometry_point vertices[4];

	int16_t point_indexes[300];
	uint16_t number_of_points = 0;

	int16_t final_point_indexes[300];
	uint16_t final_number_of_points = 0;

	long double center_x = 0;
	long double center_y = 0;
	long double size_x = 0;
	long double size_y = 0;

	uint8_t object_size = 0;

	Geometry_point distant_point = {0, 5000};

	uint16_t counter = 0;

	void calculate_geomety(){
		object_size = 0;

		center_x = 0;
		center_y = 0;
		size_x = 0;
	 	size_y = 0;
		if (final_number_of_points > POINTS_THRESHOLD){
			//printf("%d \n", subzone_number_of_points[i]);
			for (int i=0; i<final_number_of_points; i++){
				center_x += assembled_points[final_point_indexes[i]].x;
				center_y += assembled_points[final_point_indexes[i]].y;
			}

			center_x = center_x / final_number_of_points;
			center_y = center_y / final_number_of_points;

			for (int i=0; i<final_number_of_points; i++){
				size_x += pow(double(abs(center_x - assembled_points[final_point_indexes[i]].x)), 0.5);
				size_y += pow(double(abs(center_y - assembled_points[final_point_indexes[i]].y)), 0.5);
			}

			size_x = pow(size_x / final_number_of_points, 2);
			size_y = pow(size_y / final_number_of_points, 2);

			if ((size_x + size_y) / 2.0 > 8){
				object_size = 1;
			}
			if ((size_x + size_y) / 2.0 > 10){
				object_size = 2;
			}
			if ((size_x + size_y) / 2.0 > 15){
				object_size = 3;
			}
		}
	}

	bool orientation(Geometry_point p, Geometry_point q, Geometry_point r){
		return ((q.y - p.y)*(r.x-q.x) - (q.x - p.x) * (r.y - q.y)) > 0;
	}

	void advanced_sort(){
		final_number_of_points = 0;

		for(int i=0; i<number_of_points; i++){
			counter = 0;
			for(int j=0; j<4; j++){
				if(orientation(vertices[j], vertices[(j+1)%4], distant_point) != orientation(vertices[j], vertices[(j+1)%4], assembled_points[point_indexes[i]]) &&
				orientation(distant_point, assembled_points[point_indexes[i]], vertices[j]) != orientation(distant_point, assembled_points[point_indexes[i]], vertices[(j+1)%4])){
					counter ++;
				}
			}

			if(counter % 2 == 1){
				final_point_indexes[final_number_of_points] = point_indexes[i];
				final_number_of_points ++;
			}
		}
	}


	void analyse_points(){
		number_of_points = 0;

		for(int i=0; i<assembled_points_amount; i++){
			if(assembled_points[i].x > bounding_rect[0].x && assembled_points[i].x < bounding_rect[1].x && assembled_points[i].y < bounding_rect[0].y && assembled_points[i].y > bounding_rect[1].y){
				point_indexes[number_of_points] = i;
				number_of_points ++;
			}
		}

		advanced_sort();
		//printf("%d %d\n", number_of_points, final_number_of_points);
		calculate_geomety();
		//printf("%d %f %f %f %f %d\n", HAL_GetTick() - main_calculation_timer, center_x, center_y, size_x, size_y, object_size);

	}
};

class Collision_detector{
public:
	Geometry_point bounding_rect[2];

	int16_t point_indexes[300];
	uint16_t number_of_points = 0;
	
	long double center_x = 0;
	long double center_y = 0;
	long double size_x = 0;
	long double size_y = 0;

	uint8_t object_exist[3] = {0, 0, 0};
	long double object_center[3] = {0, 0, 0};

	uint32_t measurment_time[3] = {0, 0, 0};

	long double velocity, projected_location1, projected_location2;

	int16_t critical_distance = 0;
	uint8_t comparison_type = 0;

	uint8_t warning[3] = {0, 0, 0};
	uint8_t braking[3] = {0, 0, 0};

	uint8_t warning_filtered = 0;
	uint8_t braking_filtered = 0;

	uint8_t counter = 0;

	void add_next_value(){
		object_exist[0] = object_exist[1];
		object_center[0] = object_center[1];
		measurment_time[0] = measurment_time[1];

		warning[0] = warning[1];
		warning[1] = warning[2];

		braking[0] = braking[1];
		braking[1] = braking[2];
	}

	void kinematics(){
		if(object_exist[0] && object_exist[1]){
			velocity = (object_center[1] - object_center[0])/(measurment_time[1] - measurment_time[0]);

			projected_location1 = object_center[1] + velocity*collision_detection_time;

			if ((projected_location1 < critical_distance && comparison_type == 0) || (projected_location1 > critical_distance && comparison_type == 1)){
				warning[2] = 1;
			}
			else{
				warning[2] = 0;
			}

			projected_location2 = object_center[1] + velocity*1000;

			if ((projected_location2 < critical_distance && comparison_type == 0) || (projected_location1 > critical_distance && comparison_type == 1)){
				braking[2] = 1;
			}
			else{
				braking[2] = 0;
			}

			//printf("%f %f %f %f %d %d %d %d %d \n", object_center[1], velocity, projected_location1, projected_location2, warning[0], warning[1], warning[2], warning_filtered, braking_filtered);
		}
		else{
			braking[2] = 0;
			warning[2] = 0;
		}
	}

	void filter(){
		counter = 0;
		for (int i=0; i<3; i++){
			counter += warning[i];
		}

		if (counter >= 2){
			warning_filtered = 1;
		}
		else{
			warning_filtered = 0;
		}

		counter = 0;
		for (int i=0; i<3; i++){
			counter += braking[i];
		}

		if (counter >= 2){
			braking_filtered = 1;
		}
		else{
			braking_filtered = 0;
		}
	}

	void calculate_cloud_center(){
		number_of_points = 0;

		for(int i=0; i<assembled_points_amount; i++){
			if(assembled_points[i].x > bounding_rect[0].x && assembled_points[i].x < bounding_rect[1].x && assembled_points[i].y < bounding_rect[0].y && assembled_points[i].y > bounding_rect[1].y){
				point_indexes[number_of_points] = i;
				number_of_points ++;
			}
		}

		//printf("%d ", number_of_points);

		add_next_value();

		if (number_of_points > POINTS_THRESHOLD){
			for (int i=0; i<number_of_points; i++){
				center_x += assembled_points[point_indexes[i]].x;
				center_y += assembled_points[point_indexes[i]].y;
			}

			center_x = center_x / number_of_points;
			center_y = center_y / number_of_points;

			for (int i=0; i<number_of_points; i++){
				size_x += pow(double(abs(center_x - assembled_points[point_indexes[i]].x)), 0.5);
				size_y += pow(double(abs(center_y - assembled_points[point_indexes[i]].y)), 0.5);
			}

			size_x = pow(size_x / number_of_points, 2);
			size_y = pow(size_y / number_of_points, 2);

			
			if ((size_x + size_y)/2.0 > 3){
				object_exist[1] = 1;
				measurment_time[1] = HAL_GetTick();
				object_center[1] = center_y;
			}
			else{
				object_exist[1] = 0;
			}
		}
		else{
			object_exist[1] = 0;
		}

		kinematics();
		filter();
	}
};

Comnplicated_zone danger_zones[2];

Zone blind_spots[5];

Collision_detector front_detector;
Collision_detector rear_detector;

struct Display_data{
	uint8_t start_byte = 0xAA;
	int8_t zones_status[7] = {-1, -1, -1, -1, -1, -1, -1};
	uint8_t collisions[2] = {0, 0};
	uint8_t lidars_status[4] = {0, 0, 0, 0};
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t power;
  	uint8_t range;
  	uint8_t sensetivity;
  	uint8_t collision;
  	uint8_t autoBreak;
  	uint8_t brightness_1;
  	uint8_t brightness_2;
};



Display_data tx_packet;
uint8_t display_tx_buffer[sizeof(tx_packet)];
uint8_t settings_tx_buffer[sizeof(settings)];



/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == UART7){
		memcpy(settings_rx_buffer ,&settings, sizeof(settings_rx_buffer));
		new_settings = 1;
		HAL_UART_Receive_IT(&huart7, settings_rx_buffer, sizeof(settings_rx_buffer));
	}
}*/

settings_profile smallest_profile;

uint32_t setting_send_timer = 0;

void loop(){
	for (int i=0; i<4; i++){
		lidars[i].tick();
	}

	if (HAL_GetTick() - main_calculation_timer > 160){

		assembled_points_amount = 0;

		for (int i=0; i<4; i++){
			//lidars[i].process_rotation();

			//lidars[i].packets_count = 0;

			/*for(int j=0; j<lidars[i].points_amount_old; j++, assembled_points_amount++){
			assembled_points[assembled_points_amount].x = (lidars[i].lidar_points_processed_old[j].x + 0.5);
			assembled_points[assembled_points_amount].y = (lidars[i].lidar_points_processed_old[j].y + 0.5);
			}*/

			for(int j=0; j<lidars[i].points_amount; j++, assembled_points_amount++){
				assembled_points[assembled_points_amount].x = int(lidars[i].lidar_points_processed[j].x + 0.5);
				assembled_points[assembled_points_amount].y = int(lidars[i].lidar_points_processed[j].y + 0.5);
			}
		}

		front_detector.calculate_cloud_center();
		rear_detector.calculate_cloud_center();

		for(int i=0; i<5; i++){
			blind_spots[i].analyse_points();
		}
		for(int i=0; i<2; i++){
			danger_zones[i].analyse_points();
		}

		tx_packet.zones_status[0] = blind_spots[0].object_zone;
		tx_packet.zones_status[1] = danger_zones[0].object_size;
		tx_packet.zones_status[2] = danger_zones[1].object_size;
		tx_packet.zones_status[3] = blind_spots[1].object_zone;
		tx_packet.zones_status[4] = blind_spots[2].object_zone;
		tx_packet.zones_status[5] = blind_spots[3].object_zone;
		tx_packet.zones_status[6] = blind_spots[4].object_zone;

		tx_packet.collisions[0] = rear_detector.warning_filtered;
		tx_packet.collisions[1] = front_detector.warning_filtered;

		for(int i=0; i<4; i++){
			if(lidars[i].normal_working == 1){
				tx_packet.lidars_status[i] = 2;
			}
			else if(lidars[i].connected == 1){
				tx_packet.lidars_status[i] = 1;
			}
			else{
				tx_packet.lidars_status[i] = 0;
			}
		}

		if(settings.autoBreak == 1 && (front_detector.braking_filtered == 1)){
			HAL_GPIO_WritePin(BRAKE_2_GPIO_Port, BRAKE_2_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(BRAKE_2_GPIO_Port, BRAKE_2_Pin, GPIO_PIN_RESET);
		}

		memcpy(display_tx_buffer, &tx_packet, sizeof(tx_packet));

		HAL_UART_Transmit_IT(&huart7, (uint8_t*)display_tx_buffer, sizeof(display_tx_buffer));

		printf("%d %d %d %d %d %d %d %d\n", settings.power, settings.sensetivity, settings.range, settings.collision, settings.autoBreak, settings.brightness_1, settings.brightness_2, HAL_GetTick() - main_calculation_timer, HAL_GetTick());

		main_calculation_timer = HAL_GetTick();
	}

	if(HAL_GetTick() - connect_update_timer > 100){
		for (int i=0; i<4; i++){
			lidars[i].update_connection_status();
		}
		
		connect_update_timer = HAL_GetTick();
	}

	if (new_settings && HAL_GetTick()>1000){
		tx_packet.sensetivity = settings.sensetivity;
		tx_packet.range = settings.range;
		tx_packet.collision = settings.collision;
		tx_packet.power = settings.power;
		tx_packet.autoBreak = settings.autoBreak;
		tx_packet.brightness_1 = settings.brightness_1;
		tx_packet.brightness_2 = settings.brightness_2;

		if (settings.sensetivity == 1){
			size_treshold = 3.5;
		}
		else if (settings.sensetivity == 2){
			size_treshold = 8;
		}
		else if (settings.sensetivity == 3){
			size_treshold = 10;
		}

		collision_detection_time = settings.collision * 1000;

		new_settings = 0;

		printf("%d %d %d %d %d %d %d %d\n", settings.power, settings.sensetivity, settings.range, settings.collision, settings.autoBreak, settings.brightness_1, settings.brightness_2, HAL_GetTick());

		memcpy(&first_reg, &settings, 4);
		memcpy(&second_reg, &settings+4, 3);
		HAL_PWR_EnableBkUpAccess();
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, first_reg);
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, second_reg);
		HAL_PWR_DisableBkUpAccess();
	}
	else if(HAL_GetTick() < 1000){
		new_settings = 0;
	}
}

void setup(){
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


	first_reg = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	second_reg = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

	memcpy(&settings, &first_reg, 4);
	memcpy(&settings+4, &second_reg, 3);

	if (settings.sensetivity == 0){
		settings.power = 0;
		settings.sensetivity = 2;
		settings.range = 2;
		settings.collision = 2;
		settings.autoBreak = 0;
		settings.brightness_1 = 2;
		settings.brightness_2 = 3;
	}

	tx_packet.sensetivity = settings.sensetivity;
	tx_packet.range = settings.range;
	tx_packet.collision = settings.collision;
	tx_packet.power = settings.power;
	tx_packet.autoBreak = settings.autoBreak;
	tx_packet.brightness_1 = settings.brightness_1;
	tx_packet.brightness_2 = settings.brightness_2;

	printf("%d %d %d %d %d %d %d\n", settings.power, settings.sensetivity, settings.range, settings.collision, settings.autoBreak, settings.brightness_1, settings.brightness_2);

  	lidars[0].lidar_index = 1;
  	lidars[1].lidar_index = 2;
  	lidars[2].lidar_index = 3;
  	lidars[3].lidar_index = 4;

  	lidars[0].y_shift = -240;
  	lidars[1].y_shift = -240;
  	lidars[2].y_shift = 240;
  	lidars[3].y_shift = 240;

  	lidars[0].invert = -1;
  	lidars[1].invert = -1;
  	lidars[2].invert = 1;
  	lidars[3].invert = 1;

	/*test_zone.bounding_rect[0] = {0, 1200};
	test_zone.bounding_rect[1] = {1000, 300};

	test_zone.zone_rects[0][0] = {0, 900};
	test_zone.zone_rects[0][1] = {750, 300};

	test_zone.zone_rects[1][0] = {0, 600};
	test_zone.zone_rects[1][1] = {250, 300};

	right_danger_zone.bounding_rect[0] = {170, 848};
	right_danger_zone.bounding_rect[1] = {623, -155};

	right_danger_zone.vertices[0] = {170, 451};
	right_danger_zone.vertices[1] = {170, -155};
	right_danger_zone.vertices[2] = {825, 623};
	right_danger_zone.vertices[3] = {825, 848};*/

	front_detector.bounding_rect[0] = {-150, 2000};
	front_detector.bounding_rect[1] = {150, 300};
	front_detector.critical_distance = 300;
	front_detector.comparison_type = 0;

	rear_detector.bounding_rect[0] = {-150, -300};
	rear_detector.bounding_rect[1] = {150, -1500};
	rear_detector.critical_distance = -300;
	rear_detector.comparison_type = 1;

	smallest_profile.blind_spot_bounding_rects[0][0] = {170, 800};
	smallest_profile.blind_spot_bounding_rects[0][1] = {550, 330};
	smallest_profile.blind_spot_zones_rects[0][0][0] = {170, 630};
	smallest_profile.blind_spot_zones_rects[0][0][1] = {430, 330};
	smallest_profile.blind_spot_zones_rects[0][1][0] = {170, 480};
	smallest_profile.blind_spot_zones_rects[0][1][1] = {300, 330};

	smallest_profile.blind_spot_bounding_rects[1][0] = {-550, 800};
	smallest_profile.blind_spot_bounding_rects[1][1] = {-170, 330};
	smallest_profile.blind_spot_zones_rects[1][0][0] = {-430, 630};
	smallest_profile.blind_spot_zones_rects[1][0][1] = {-170, 330};
	smallest_profile.blind_spot_zones_rects[1][1][0] = {-300, 480};
	smallest_profile.blind_spot_zones_rects[1][1][1] = {-170, 330};

	smallest_profile.blind_spot_bounding_rects[2][0] = {-550, -330};
	smallest_profile.blind_spot_bounding_rects[2][1] = {-170, -480};
	smallest_profile.blind_spot_zones_rects[2][0][0] = {-430, -330};
	smallest_profile.blind_spot_zones_rects[2][0][1] = {-300, -630};
	smallest_profile.blind_spot_zones_rects[2][1][0] = {-300, -330};
	smallest_profile.blind_spot_zones_rects[2][1][1] = {-170, -800};

	smallest_profile.blind_spot_bounding_rects[3][0] = {-170, -330};
	smallest_profile.blind_spot_bounding_rects[3][1] = {170, -480};
	smallest_profile.blind_spot_zones_rects[3][0][0] = {-170, -330};
	smallest_profile.blind_spot_zones_rects[3][0][1] = {170, -630};
	smallest_profile.blind_spot_zones_rects[3][1][0] = {-170, -330};
	smallest_profile.blind_spot_zones_rects[3][1][1] = {170, -800};

	smallest_profile.blind_spot_bounding_rects[4][0] = {170, -330};
	smallest_profile.blind_spot_bounding_rects[4][1] = {550, -480};
	smallest_profile.blind_spot_zones_rects[4][0][0] = {170, -330};
	smallest_profile.blind_spot_zones_rects[4][0][1] = {430, -630};
	smallest_profile.blind_spot_zones_rects[4][1][0] = {170, -330};
	smallest_profile.blind_spot_zones_rects[4][1][1] = {300, -800};

	smallest_profile.dadnger_zones_bounding_rects[0][0] = {170, 565};
	smallest_profile.dadnger_zones_bounding_rects[0][1] = {550, -155};
	smallest_profile.danger_zones_vertices[0][0] = {170, 451};
	smallest_profile.danger_zones_vertices[0][1] = {550, 556};
	smallest_profile.danger_zones_vertices[0][2] = {550, 405};
	smallest_profile.danger_zones_vertices[0][3] = {170, -155};

	smallest_profile.dadnger_zones_bounding_rects[1][0] = {-550, 565};
	smallest_profile.dadnger_zones_bounding_rects[1][1] = {-170, -155};
	smallest_profile.danger_zones_vertices[1][0] = {-170, 451};
	smallest_profile.danger_zones_vertices[1][1] = {-550, 556};
	smallest_profile.danger_zones_vertices[1][2] = {-550, 405};
	smallest_profile.danger_zones_vertices[1][3] = {-170, -155};

	for(int i=0; i < 5; i++){
		blind_spots[i].bounding_rect[0] = smallest_profile.blind_spot_bounding_rects[i][0];
		blind_spots[i].bounding_rect[1] = smallest_profile.blind_spot_bounding_rects[i][1];

		blind_spots[i].zone_rects[0][0] = smallest_profile.blind_spot_zones_rects[i][0][0];
		blind_spots[i].zone_rects[0][1] = smallest_profile.blind_spot_zones_rects[i][0][1];
		blind_spots[i].zone_rects[1][0] = smallest_profile.blind_spot_zones_rects[i][1][0];
		blind_spots[i].zone_rects[1][1] = smallest_profile.blind_spot_zones_rects[i][1][1];
	}

	for(int i=0; i < 2; i++){
		danger_zones[i].bounding_rect[0] = smallest_profile.dadnger_zones_bounding_rects[i][0];
		danger_zones[i].bounding_rect[1] = smallest_profile.dadnger_zones_bounding_rects[i][1];

		danger_zones[i].vertices[0] = smallest_profile.danger_zones_vertices[i][0];
		danger_zones[i].vertices[1] = smallest_profile.danger_zones_vertices[i][1];
		danger_zones[i].vertices[2] = smallest_profile.danger_zones_vertices[i][2];
		danger_zones[i].vertices[3] = smallest_profile.danger_zones_vertices[i][3];
	}


  	/*HAL_GPIO_WritePin(CONNECT_LED_1_GPIO_Port, CONNECT_LED_1_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(CONNECT_LED_2_GPIO_Port, CONNECT_LED_2_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(CONNECT_LED_3_GPIO_Port, CONNECT_LED_3_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(CONNECT_LED_4_GPIO_Port, CONNECT_LED_4_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(WORKING_LED_1_GPIO_Port, WORKING_LED_1_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(WORKING_LED_2_GPIO_Port, WORKING_LED_2_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(WORKING_LED_3_GPIO_Port, WORKING_LED_3_Pin, GPIO_PIN_SET);
  	HAL_GPIO_WritePin(WORKING_LED_4_GPIO_Port, WORKING_LED_4_Pin, GPIO_PIN_SET);*/
  	printf("%d ", HAL_GPIO_ReadPin(CONNECT_DET_1_GPIO_Port, CONNECT_DET_1_Pin));
  	printf("%d ", HAL_GPIO_ReadPin(CONNECT_DET_2_GPIO_Port, CONNECT_DET_2_Pin));
  	printf("%d ", HAL_GPIO_ReadPin(CONNECT_DET_3_GPIO_Port, CONNECT_DET_3_Pin));
  	printf("%d \n", HAL_GPIO_ReadPin(CONNECT_DET_4_GPIO_Port, CONNECT_DET_4_Pin));


  	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, lidars[0].RxData, BUFFER_LENGTH);
  	memset(lidars[0].RxData, '\0', BUFFER_LENGTH);

  	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, lidars[1].RxData, BUFFER_LENGTH);
  	memset(lidars[1].RxData, '\0', BUFFER_LENGTH);

  	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, lidars[2].RxData, BUFFER_LENGTH);
  	memset(lidars[2].RxData, '\0', BUFFER_LENGTH);

  	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, lidars[3].RxData, BUFFER_LENGTH);
  	memset(lidars[3].RxData, '\0', BUFFER_LENGTH);

	HAL_UARTEx_ReceiveToIdle_IT(&huart7, settings_rx_buffer, sizeof(settings_rx_buffer));
}
