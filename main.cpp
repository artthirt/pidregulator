#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <sstream>
#include <stdio.h>
#include <random>
#include <signal.h>
#include <unistd.h>
#include <atomic>

#include <X11/Xlib.h>
#include <sys/resource.h>
#include <pthread.h>
#include <random>

#include <opencv2/opencv.hpp>

using namespace std;

void _sleep(int64_t ms)
{
	timespec ts;
	ts.tv_nsec = (ms * 1000000) % (int64_t)(1e+9);
	ts.tv_sec = ms / 1000;
	clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, 0);
}

const double mean = 0;
const double sgm = 3;

std::atomic<bool> done(false);

cv::Vec2d operator* (const cv::Vec2d& v1, const cv::Vec2d & v2)
{
	cv::Vec2d res;
	res[0] = v1[0] * v2[0];
	res[1] = v1[1] * v2[1];
	return res;
}

namespace automatic_control{

/**
 * @brief The pidregulator struct
 * calculation for PID automatic control
 */
template< class T = double >
struct pidregulator{
	double dt;
	T current;
	T needed;
	T prev_error;
	T intg;			/// integreal value
	T coeff0;		/// coefficient for summator
	T coeff1;		/// coefficient for protoprtional part
	T coeff2;		/// coefficient for integral part
	T coeff3;		/// coefficient for diffirential part
	int counter;

	pidregulator(){
		init();
	}
	/**
	 * @brief pidregulator
	 * @param n
	 * @param c0	- summator
	 * @param c1	- proportional
	 * @param c2	- integral
	 * @param c3	- diffirential
	 */
	pidregulator(T n, T c0, T c1, T c2, T c3){
		init();
		needed = n;
		coeff0 = c0;
		coeff1 = c1;
		coeff2 = c2;
		coeff3 = c3;
	}
	void init(){
		dt			= 1;
		counter		= 0;
		current		= T();
		needed		= T();
		intg		= T();
		prev_error	= T();
		//coeff0 = coeff1 = coeff2 = coeff3 = T();
	}
	/**
	 * @brief error
	 * @return
	 */
	inline T error(){
		return needed - current;
	}
	/**
	 * @brief proportional
	 * proportional part
	 * @return
	 */
	inline T proportional(){
		return coeff1 * error();
	}
	/**
	 * @brief integral
	 * integral part
	 * @return
	 */
	inline T integral(){
		intg += error() * dt;
		return coeff2 * intg;
	}
	/**
	 * @brief diff
	 * differential part
	 * @return
	 */
	inline T diff(){
		double e = error();
		T res =  coeff3 * (e - prev_error);
		prev_error = e;
		return res;
	}
	/**
	 * @brief calc
	 * calc PID regulator
	 */
	T calc(T current, T needed, double dt){

		this->dt		= dt;
		this->current	= current;
		this->needed	= needed;

		T res, res1, res2, res3;
		res1 = proportional();
		res2 = integral();
		res3 = diff();
		res = coeff0 * (res1 + res2 + res3);
		current = res;

		cout << counter << "\tc[" << res << "]\tp[" << res1 << "]\ti[" << res2 << "]\td[" << res3 << "]\n";

		counter++;
		return res;
	}
	void set_current(T value){
		current = value;
	}
};

}

struct generate_value{
	std::normal_distribution<double> distr;
	std::random_device rd;

	generate_value(){
		distr = std::normal_distribution<double>(mean, sgm);
	}

	double operator()(){
		return distr(rd);
	}

};

generate_value genval;

//void loop_time(automatic_control::pidregulator<cv::Vec2d>& pid)
//{
//	cv::Vec2d rnd = cv::Vec2d(genval(), genval());
//	//speed += rnd;
//	pid.calc(rnd);
//}

void handler(int s)
{
	done = true;
	printf("app will close\n");
	exit(1);
}

bool getkey_val(char* keys, int key)
{
	int ind1 = key / 8;
	int ind2 = key % 8;
	return (keys[ind1] & ind2) != 0;
}

void* pthread_run(void* user)
{
	char keys[32] = { 0 };

	Display *display = 0;

	display = XOpenDisplay(":0");

	if(!display){
		std::cout << "display not open\n";
		return 0;
	}

	while(!done){
		XQueryKeymap(display, keys);
		if(getkey_val(keys, 10)){
			done = true;
		}

		usleep(100 * 1000);
	}

	XCloseDisplay(display);
}

const string name_wnd("graph");
const size_t count_vec = 100;

typedef double (func)(double t);

struct widget{

	bool trigger_mouse		= false;
	double max_y			= 20;
	double min_y			= -10;
	double multiply_tb		= 40;
	double min_value_tb		= -10;
	double max_u			= 10;
	double min_u			= -10;
	double grav				= 9.81;
	double mass				= 1;
	double koeff			= 1;
	int max_trackbar_val	= 1000;
	double max_t			= 5.;
	vector<double> data;
	vector<double> data_needed;
	automatic_control::pidregulator<double> pid;
	std::normal_distribution< double > rnd;
	std::default_random_engine generator_rnd;
	cv::Mat mat;

	func *fnc;

	static double sample_f(double d){
		return 0;
	}

	widget(){
		fnc = &widget::sample_f;
		rnd = normal_distribution<double>(0, 0.03);
	}

	void generate(func *f){
		pid.init();
		data.resize(0);
		data_needed.resize(0);
		double val = 0;
		double val_speed = 0;
		double val_accel = 0;
		double dt = max_t / count_vec;
		for(size_t i = 0; i < count_vec; i++){
			double t = i * dt;
			double val_n = (*f)(t);
			double val_u = koeff * pid.calc(val, val_n, dt);
			if(val_u > max_u)
				val_u = max_u;
			if(val_u < min_u)
				val_u = min_u;
			val_accel =  mass * (val_u + grav) * dt;
			val_speed += val_accel;
			val_speed -= mass * grav * dt;
			//val_speed += rnd(generator_rnd);
			val += val_speed;
//			if(val < 0){
//				val = 0;
//				val_speed = 0;
//				val_accel = 0;
//			}
			data.push_back(val);
			data_needed.push_back(val_n);
		}

		mat = cv::Mat::zeros(800, 800, CV_8UC3);

		double x = 0, delta = (double)mat.cols / data.size();
		double dy = max_y - min_y;

		for(size_t i = 1; i < data.size(); i++){
			double d1 = data[i - 1];
			double d2 = data[i];
			double y1 = (d1 - min_y)/dy * mat.rows;
			double y2 = (d2 - min_y)/dy * mat.rows;

			cv::Point p1(x, mat.rows - y1), p2(x + delta, mat.rows - y2);

			cv::line(mat, p1, p2, cv::Scalar(255, 255, 255));
			x += delta;
		}
		x = 0;
		for(size_t i = 1; i < data_needed.size(); i++){
			double d1 = data_needed[i - 1];
			double d2 = data_needed[i];
			double y1 = (d1 - min_y)/dy * mat.rows;
			double y2 = (d2 - min_y)/dy * mat.rows;

			cv::Point p1(x, mat.rows - y1), p2(x + delta, mat.rows - y2);

			cv::line(mat, p1, p2, cv::Scalar(0, 0, 255));
			x += delta;
		}
	}


	void show_wnd()
	{
		if(mat.empty()){
			cv::namedWindow(name_wnd);

			cv::createTrackbar("C0", name_wnd, 0, max_trackbar_val, widget::change_C0, this);
			cv::createTrackbar("P", name_wnd, 0, max_trackbar_val, widget::change_P, this);
			cv::createTrackbar("I", name_wnd, 0, max_trackbar_val, widget::change_I, this);
			cv::createTrackbar("D", name_wnd, 0, max_trackbar_val, widget::change_D, this);

			cv::setTrackbarPos("C0", name_wnd, (pid.coeff0 - min_value_tb) * max_trackbar_val / multiply_tb);
			cv::setTrackbarPos("P", name_wnd, (pid.coeff1 - min_value_tb) * max_trackbar_val / multiply_tb);
			cv::setTrackbarPos("I", name_wnd, (pid.coeff2 - min_value_tb) * max_trackbar_val / multiply_tb);
			cv::setTrackbarPos("D", name_wnd, (pid.coeff3 - min_value_tb) * max_trackbar_val / multiply_tb);

			generate(fnc);
		}

		stringstream ss;
		ss << "c0[" << pid.coeff0 << "]; p[" << pid.coeff1 << "]; i[" << pid.coeff2 << "]; d[" << pid.coeff3 << "]";

		cv::putText(mat, ss.str(), cv::Point(10, 15), 1, 1, cv::Scalar(255, 255, 255));
		//cv::displayStatusBar(name_wnd, ss.str());

		cv::imshow(name_wnd, mat);
	}

	static void change_C0(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff0 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_P(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff1 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_I(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff2 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_D(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff3 =  w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
};

typedef double d3[3];
const d3 ranges[] = {
	  {0, 1, 0}
	, {1, 2, 2}
	, {2, 2.5, 7}
	, {2.5, 3, 2}
	, {3, 4, 5}
	, {4, 100, 3}
};

double f_t(double t)
{
	int cnt = sizeof(ranges)/sizeof(*ranges);
	for(int i = 0; i < cnt; i++){
		if(t >= ranges[i][0] && t < ranges[i][1])
			return ranges[i][2];
	}
	return 0;
}

int main()
{
	/// hook ctrl+c
	struct sigaction sa;
	sa.sa_handler = handler;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;

	sigaction(SIGINT, &sa, 0);

	pthread_t ptr;
	pthread_attr_t pattr;
	std::fill((char*)&pattr, (char*)&pattr + sizeof(pattr), '\0');
	/// loop to get a key
	pthread_create(&ptr, &pattr, &pthread_run, 0);

	widget wg;
	wg.fnc = &f_t;
	wg.pid = automatic_control::pidregulator<double>(f_t(0), 0.4, 0.7, 0, 0);
	while(!done){
//		loop_time(pidxy);
		wg.show_wnd();
		cv::waitKey(10);
	}

	pthread_join(ptr, 0);

	return 0;
}

