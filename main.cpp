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
	T current;
	T needed;
	T prev_error;
	T prev_error2;
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
		counter		= 0;
		current		= T();
		needed		= T();
		intg		= T();
		prev_error	= T();
		prev_error2 = T();
		coeff0 = coeff1 = coeff2 = coeff3 = T();
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
		intg += error();
		return coeff2 * intg;
	}
	/**
	 * @brief diff
	 * differential part
	 * @return
	 */
	inline T diff(){
		T res =  coeff3 * (error() - 2. * prev_error + prev_error2);
		prev_error2 = prev_error;
		prev_error = error();
		return res;
	}
	/**
	 * @brief calc
	 * calc PID regulator
	 */
	void calc(){
		T res1, res2, res3;
		res1 = proportional();
		res2 = integral();
		res3 = diff();
		current += coeff0 * (res1 + res2 + res3);

		cout << counter << "\tc[" << current << "]\tp[" << res1 << "]\ti[" << res2 << "]\td[" << res3 << "]\n";

		counter++;
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

void loop_time(automatic_control::pidregulator<cv::Vec2d>& pid)
{
	cv::Vec2d rnd = cv::Vec2d(genval(), genval());
	//speed += rnd;
	pid.current += rnd;
	pid.calc();
}

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
const size_t count_vec = 50;

struct widget{

	double needed			= 100;
	bool trigger_mouse		= false;
	double max_y			= 200;
	double min_y			= -100;
	double multiply_tb		= 3;
	double min_value_tb		= -1;
	int max_trackbar_val	= 1000;
	vector<double> data;
	automatic_control::pidregulator<double> pid;
	cv::Mat mat;

	widget(){

	}

	void generate(){
		pid.current = 0;
		pid.needed = 100;
		pid.intg = 0;
		data.resize(0);
		for(size_t i = 0; i < count_vec; i++){
			data.push_back(pid.current);
			pid.calc();
		}

		mat = cv::Mat::zeros(800, 800, CV_8UC1);

		double x = 0, delta = mat.cols / data.size();
		double dy = max_y - min_y;

		for(size_t i = 1; i < data.size(); i++){
			double d1 = data[i - 1];
			double d2 = data[i];
			double y1 = (d1 - min_y)/dy * mat.rows;
			double y2 = (d2 - min_y)/dy * mat.rows;

			cv::Point p1(x, mat.rows - y1), p2(x + delta, mat.rows - y2);

			cv::line(mat, p1, p2, cv::Scalar(255));
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

			generate();
		}

		stringstream ss;
		ss << "c0[" << pid.coeff0 << "]; p[" << pid.coeff1 << "]; i[" << pid.coeff2 << "]; d[" << pid.coeff3;

		cv::putText(mat, ss.str(), cv::Point(10, 15), 1, 1, cv::Scalar(255));
		//cv::displayStatusBar(name_wnd, ss.str());

		cv::imshow(name_wnd, mat);
	}

	static void change_C0(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff0 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate();
	}
	static void change_P(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff1 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate();
	}
	static void change_I(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff2 = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate();
	}
	static void change_D(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeff3 =  w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate();
	}
};

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
	wg.pid = automatic_control::pidregulator<double>(100, 0.3, 0.7, 0.001, 0.28);
	while(!done){
//		loop_time(pidxy);
		wg.show_wnd();
		cv::waitKey(10);
	}

	pthread_join(ptr, 0);

	return 0;
}

