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
const double sgm = 0.7;

std::random_device rd;
std::atomic<bool> done(false);

namespace automatic_control{

/**
 * @brief The pidregulator struct
 * calculation for PID automatic control
 */
struct pidregulator{
	double current;
	double speed;
	double needed;
	double prev_error;
	double intg;		/// integreal value
	double coeff0;		/// coefficient for summator
	double coeff1;		/// coefficient for protoprtional part
	double coeff2;		/// coefficient for integral part
	double coeff3;		/// coefficient for diffirential part
	int counter;
	std::normal_distribution<double> distr;

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
	pidregulator(double n, double c0, double c1, double c2, double c3){
		init();
		needed = n;
		coeff0 = c0;
		coeff1 = c1;
		coeff2 = c2;
		coeff3 = c3;
	}
	void init(){
		current		= 0;
		needed		= 0;
		intg		= 0;
		speed		= 0;
		counter		= 0;
		prev_error	= 0;
		coeff0 = coeff1 = coeff2 = coeff3 = 0;
		distr = std::normal_distribution<double>(mean, sgm);
	}
	/**
	 * @brief error
	 * @return
	 */
	inline double error(){
		return needed - current;
	}
	/**
	 * @brief proportional
	 * proportional part
	 * @return
	 */
	inline double proportional(){
		return coeff1 * error();
	}
	/**
	 * @brief integral
	 * integral part
	 * @return
	 */
	inline double integral(){
		intg += error();
		return coeff2 * intg;
	}
	/**
	 * @brief diff
	 * differential part
	 * @return
	 */
	inline double diff(){
		double res =  coeff3 * (error() - prev_error);
		prev_error = error();
		return res;
	}
	/**
	 * @brief calc
	 * calc PID regulator
	 */
	void calc(){
		double res1, res2, res3;
		res1 = proportional();
		res2 = integral();
		res3 = diff();
		speed += coeff0 * (res1 + res2 + res3);

		char buf[512];
		sprintf(buf, "%d\tc[%f]\tp[%f]\ti[%f]\td[%f]\tspd[%f]", counter, current,
				res1, res2, res3, speed);
		cout << buf << endl;
		counter++;
	}
	/**
	 * @brief new_current
	 */
	void new_current(){
		double rnd = distr(rd);
		speed += rnd;
		current += speed;
	}
};

}

void loop_time(automatic_control::pidregulator& pid)
{
	pid.new_current();
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

cv::Point needed_pt(100, 100);
cv::Point pt_old(0, 0);
bool trigger_mouse = false;


void show_wnd(cv::Mat& mat, automatic_control::pidregulator& pidx, automatic_control::pidregulator& pidy)
{
	if(mat.empty()){
		mat = cv::Mat::zeros(800, 800, CV_8UC1);
	}

	if(trigger_mouse){
		pidx.needed = needed_pt.x;
		pidy.needed = needed_pt.y;
		trigger_mouse = false;
	}

	cv::Point pt(pidx.current, pidy.current);

	cv::line(mat, pt_old, pt, cv::Scalar(255));

	pt_old = pt;

	pt.x %= mat.cols;
	if(pt.y >=0 && pt.y < mat.rows){
		mat.at<u_char>(pt.y, pt.x) = 255;
	}

	cv::imshow(name_wnd, mat);
}

void mouse_clbck(int event, int x, int y, int flags, void* userdata)
{
	if(event == cv::EVENT_LBUTTONDOWN){
		needed_pt = cv::Point(x, y);
		trigger_mouse = true;
	}
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

	cv::namedWindow(name_wnd, cv::WINDOW_NORMAL);
	cv::setMouseCallback(name_wnd, mouse_clbck);
	cv::Mat grph_out;

	automatic_control::pidregulator pidx(100, 0.9, 0.19, 0.001, 0.95);
	automatic_control::pidregulator pidy(100, 0.9, 0.19, 0.001, 0.95);
	while(!done){
		loop_time(pidx);
		loop_time(pidy);
		show_wnd(grph_out, pidx, pidy);
		_sleep(30);
		cv::waitKey(1);
	}

	pthread_join(ptr, 0);

	return 0;
}

