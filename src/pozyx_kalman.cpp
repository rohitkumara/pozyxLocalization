#include <bits/stdc++.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

//OpenCV
#include <opencv2/opencv.hpp>

//Serial Communication
extern "C"
{
	#include "serial_lib.h"
}


struct Quaternion{
	float w;
	float x;
	float y;
	float z;
};

struct vect{
	float x;
	float y;
	float z;
};

void quat_rotate(struct Quaternion *q, struct vect *v){
	q->w = q->w / 16384.0;
	q->x = q->x / 16384.0;
	q->y = q->y / 16384.0;
	q->z = q->z / 16384.0;

	float norm = sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->y*q->y);

	q->w = q->w / norm;
	q->x = q->x / norm;
	q->y = q->y / norm;
	q->z = q->z / norm;

	float angle = acos(q->w);

	float q00 = 2.0 * q->x * q->x;
	float q11 = 2.0 * q->y * q->y;
	float q22 = 2.0 * q->z * q->z;

	float q01 = 2.0 * q->x * q->y;
	float q02 = 2.0 * q->x * q->z;
	float q03 = 2.0 * q->x * q->w;

	float q12 = 2.0 * q->y * q->z;
	float q13 = 2.0 * q->y * q->w;

	float q23 = 2.0 * q->z * q->w;

	float tmp_x = (1.0 - q11 - q22) * v->x + (q01 - q23) * v->y + (q02 + q13) * v->z;
	float tmp_y = (q01 + q23) * v->x + (1.0 - q22 - q00) * v->y + (q12 - q03) * v->z;
	float tmp_z = (q02 - q13) * v->x + (q12 + q03) * v->y + (1.0 - q11 - q00) * v->z;

	//v->y = tmp_y;
	//v->x = tmp_x;

	v->y = cos(angle)*tmp_x - sin(angle)*tmp_y;
	v->x = sin(angle)*tmp_x + cos(angle)*tmp_y;
	
	v->z = tmp_z;
}

std::vector<std::string> tokenize(std::string str){
	int pos = 0;
	std::vector<std::string> v;
	while(true){
		int p1 = str.find(",", pos);
		if(p1 == -1){
			std::string dat = str.substr(pos);
			v.push_back(dat);
			break;
		}
		std::string dat = str.substr(pos, (p1-pos));
		v.push_back(dat);
		pos = p1+1;
	}
	return v;
}


void init_kalman(cv::KalmanFilter &kalman, double dt){
	//state transition matrix (x, y, vx, vy)
	cv::Mat1f F(4,4);
	F << 1.0, 0.0,  dt, 0.0,
		 0.0, 1.0, 0.0,  dt,
		 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0;
	kalman.transitionMatrix = F;

	//measurement matrix (x, y, 0, 0)
	cv::Mat1f H(2,4);
	H << 1.0, 0.0, 0.0, 0.0,
		 0.0, 1.0, 0.0, 0.0;
	kalman.measurementMatrix = H;

	//Process noise covariance (x, y, vx, vy)
	cv::Mat1f Q(4,4);
	Q << 0.1, 0.0, 0.0, 0.0,
		 0.0, 0.1, 0.0, 0.0,
		 0.0, 0.0, 0.3, 0.0,
		 0.0, 0.0, 0.0, 0.3;
	kalman.processNoiseCov = Q;

	// Measurement Noise covariance (x, y, 0, 0)
	cv::Mat1f R(2,2);
	R << 10, 0.0,
		 0.0, 10;
	kalman.measurementNoiseCov = R;

	// Control Matrix
	cv::Mat1f B(4,2);
	B << 0.5*dt*dt, 	  0.0,
		 		 0, 0.5*dt*dt,
		 		dt, 		0,
		 		 0,		   dt;
	kalman.controlMatrix = B;
}

void init_anchors(int fd ,char buf[], char eolchar, int buff, int timeout){
	int num_a = 0;
	while(true){
		serialport_read_until(fd, buf, eolchar, buff, timeout);
		std::string anc = "Anchors found:";
		std::string dat = std::string(buf);
		if(dat.find(anc) != std::string::npos){
			num_a = dat[dat.length()-2] - '0';
			break;
		}
	}
	std::vector<int> anchor_names(num_a);
	std::vector< std::vector<int> > anchors(num_a);
	int i=0;
	while(i<num_a){
		std::vector<int> temp(3);
		serialport_read_until(fd, buf, eolchar, buff, timeout);
		std::string dat = std::string(buf);
		if(dat[0] == '\n')
			continue;
		std::vector<std::string> v = tokenize(dat);

		if(v[0] != "ANCHOR")
			std::cout << "Error: Did not receive ANCHOR parameters" << std::endl;
		
		for(int j=2;j<v.size();j++){
			temp[j-2] = std::stoi(v[j]);
		}
		anchors[i] = temp;
		anchor_names[i] = (int)strtol(v[1].c_str(),NULL,0);
		i++;
	}

	// for(int i=0; i<num_a; i++){
	// 	x_offset = (x_offset<anchors[i][0])?x_offset:anchors[i][0];
	// 	y_offset = (y_offset<anchors[i][1])?y_offset:anchors[i][1];
	// 	z_offset = (z_offset<anchors[i][2])?z_offset:anchors[i][2];
	// }

	// for(int i=0; i<num_a; i++){
	// 	anchors[i][0] -= x_offset;
	// 	anchors[i][1] -= y_offset;
	// 	anchors[i][2] -= z_offset;
	// }

	// int min_x = 0;
	// int min_y = 0;
	// int max_x = 0;
	// int max_y = 0;

	// for(int i=0; i<num_a; i++){
	// 	max_x = (max_x>anchors[i][0])?max_x:anchors[i][0];
	// 	max_y = (max_y>anchors[i][1])?max_y:anchors[i][1];
	// }

	// img = cv::Mat(cv::Size(max_x+100, max_y+100), CV_8UC1, cv::Scalar(255));

	// for(int i=0; i<num_a; i++){
	// 	cv::circle(img, cv::Point(anchors[i][0], anchors[i][1]), 50, cv::Scalar(0), -1);
	// 	std::cout << anchors[i][0] << " " << anchors[i][1] << std::endl;
	// }

	// for(int i=0; i<max_x; i += (max_x/grid_size)){
	// 	//cv::line(img, cv::Point(i,min_y), cv::Point(i,max_y), cv::Scalar(0), 1);
	// }

	// for(int i=0; i<max_y; i += (max_y/grid_size)){
	// 	//cv::line(img, cv::Point(min_x,i), cv::Point(max_x,i), cv::Scalar(0), 1);
	// }	
}

int main()
{
	std::string device = "/dev/ttyUSB1";

	// cv::Mat img;

	struct Quaternion *q = (struct Quaternion *)malloc(sizeof(struct Quaternion));
	struct vect *v = (struct vect *)malloc(sizeof(struct vect));

	std::ofstream log;

	const int buff = 256;

	int fd = -1;
	int baudrate = 115200;
	char quiet = 0;
	char eolchar = '\n';
	int timeout = 5000;
	char buf[buff];
	int rc,n;

	fd = serialport_init(device.c_str(), baudrate);

	if(fd == -1){
		std::cout << "Error in opening Serial Port" << std::endl;
		return -1;
	}

	memset(buf, 0, buff);

	init_anchors(fd, buf, eolchar, buff, timeout);

	// std::vector<int> old_a(3,200);
	std::vector<int> measure_a(3,0);

	char trig = 'r';

	serialport_flush(fd);

	//Kalman Filter variable init
	cv::KalmanFilter kalman(4, 2, 2); //kalman filter
	const double dt = 0.12; //loop time/sampling time (chnage as per device after calculating dt in first run)

		float av_time = 0;
		int num = 0;

		init_kalman(kalman, dt);

		std::string path = std::string("../logs/moving-") + std::to_string(0) + std::string(".log"); 

		log.open(path.c_str(), std::ofstream::out);

		auto start = std::chrono::high_resolution_clock::now();
		std::cout << "lol" << std::endl;
		for(int i=0;i<500;i++){
			serialport_write(fd ,&trig);
			serialport_read_until(fd, buf, eolchar, buff, timeout);
				
			std::string dat = std::string(buf);
			std::vector<std::string> tok = tokenize(dat);
			
			if(tok[0] != "POS")
				continue;

			q->w = stoi(tok[5]);
			q->x = stoi(tok[6]);
			q->y = stoi(tok[7]);
			q->z = stoi(tok[8]);
			
			v->x = stoi(tok[9]);
			v->y = stoi(tok[10]);
			v->z = stoi(tok[11]);

			quat_rotate(q, v);

			v->y = -v->y;
			v->z = -v->z;

			// cv::line(img, cv::Point(x_centre,y_centre+150), cv::Point(x_centre,y_centre-150), cv::Scalar(0), 2);
			// cv::line(img, cv::Point(x_centre-150,y_centre), cv::Point(x_centre+150,y_centre), cv::Scalar(0), 2);

			// cv::circle(img, cv::Point(x_centre+old_x/50, y_centre+old_y/50), 25, cv::Scalar(255), -1);
			// cv::circle(img, cv::Point(x_centre+v->x/50, y_centre+v->y/50), 25, cv::Scalar(0), -1);

			// old_x = v->x;
			// old_y = v->y;

			measure_a[0] = (std::stoi(tok[2]));
			measure_a[1] = (std::stoi(tok[3]));

			cv::Mat1f U(2,1);
			U << v->x,
				 v->y;


			cv::Mat M = kalman.predict(U); //predict step

			cv::Mat1f act_M = (cv::Mat1f(2,1) << measure_a[0], measure_a[1]);
			// std::cout << "Actual = " << act_M << std::endl;

			cv::Mat1f est_M = kalman.correct(act_M); //update step

			// std::cout << "Predict = " << est_M << std::endl;

			//cv::circle(img, cv::Point(old_a[0], old_a[1]), 50, cv::Scalar(255), -1);
			//cv::circle(img, cv::Point(est_M(0,0), est_M(1,0)), 50, cv::Scalar(0), -1);

			// old_a[0] = est_M(0,0);
			// old_a[1] = est_M(1,0);

			log << "X:" << int(est_M(0,0)) << " Y:" << int(est_M(1,0)) << "\n";

			// cv::namedWindow( "Display window", cv::WINDOW_NORMAL );
			// cv::resizeWindow("Display window", 600, 600);
			// cv::imshow( "Display window", img );
			// if(cv::waitKey(3) >= 0)
			// 	break;

			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = (end - start);
			
			// std::cout << elapsed.count() << std::endl;
			
			av_time += elapsed.count();
			num++;
			start = end;

		}
		std::cout << (float)av_time/num << " " << num << std::endl;
		log.close();	
	return 0;
}
