#include <iostream>
#include <time.h>
#include <vector>
#include <string>
#include <sstream>
#include <stdio.h>
#include <random>
#include <signal.h>
#include <atomic>
#include <omp.h>


#ifndef _MSC_VER
#include <unistd.h>
#include <X11/Xlib.h>
#include <sys/resource.h>
#include <pthread.h>
#endif

#include <opencv2/opencv.hpp>

using namespace std;

#ifndef _MSC_VER

void _sleep(int64_t ms)
{
	timespec ts;
	ts.tv_nsec = (ms * 1000000) % (int64_t)(1e+9);
	ts.tv_sec = ms / 1000;
	clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, 0);
}

#endif

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
	enum {n_coeffs = 4};
	double dt;
	T current;
	T needed;
	T prev_error;
	T intg;			/// integreal value
	T coeffs[n_coeffs];		/// coefficient for summator
	///T coeff1;				/// coefficient for protoprtional part
	///T coeff2;				/// coefficient for integral part
	///T coeff3;				/// coefficient for diffirential part
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
		coeffs[0] = c0;
		coeffs[1] = c1;
		coeffs[2] = c2;
		coeffs[3] = c3;
	}
	pidregulator(T c[]){
		init();
		for(int i = 0; i < n_coeffs; i++){
			coeffs[i] = c[i];
		}
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
	void reset(){
		current = 0;
		prev_error = 0;
		intg = 0;
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
		return coeffs[1] * error();
	}
	/**
	 * @brief integral
	 * integral part
	 * @return
	 */
	inline T integral(){
		intg += error() * dt;
		return coeffs[2] * intg;
	}
	/**
	 * @brief diff
	 * differential part
	 * @return
	 */
	inline T diff(){
		double e = error();
		T res =  coeffs[3] * (e - prev_error);
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
		res = coeffs[0] * (res1 + res2 + res3);
		current = res;

		//cout << counter << "\tc[" << res << "]\tp[" << res1 << "]\ti[" << res2 << "]\td[" << res3 << "]\n";

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

#ifndef _MSC_VER

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

#endif

const string name_wnd("graph");

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
	size_t count_vec;
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

	widget(size_t cnt_vec = 100){
		count_vec = cnt_vec;
		fnc = &widget::sample_f;
		rnd = normal_distribution<double>(0, 0.03);
	}

	void generate(func *f, int n_samples, double coeffs[], double sigmas[],
				  std::vector<double>& coeffs_output, double& error_out, double smooth_coeff){
		double dt = max_t / count_vec;

		double prev_error = 99999999999;
		double sigma_sum = 0;

		std::default_random_engine gen;

		std::vector< std::uniform_real_distribution<double> > vec_distrib;

		vec_distrib.resize(automatic_control::pidregulator<double>::n_coeffs);

		std::atomic_int counter;
		counter = 0;

		coeffs_output.resize(automatic_control::pidregulator<double>::n_coeffs);
		for(size_t i = 0; i < vec_distrib.size(); i++){
			vec_distrib[i] = std::uniform_real_distribution<double>(coeffs[i] - sigmas[i], coeffs[i] + sigmas[i]);
		}
#pragma omp parallel for shared(vec_distrib, smooth_coeff)
		for(int iter = 0; iter < n_samples; iter++){
//			cout << omp_get_thread_num() << endl;
			//vector<double> data;
			//vector<double> data_needed;

			std::vector< double > coeffs_tmp;
			coeffs_tmp.resize(automatic_control::pidregulator<double>::n_coeffs);
			for(size_t i = 0; i < vec_distrib.size(); i++){
				coeffs_tmp[i] = std::abs(vec_distrib[i](gen));
			}

			automatic_control::pidregulator<double> pid(&coeffs_tmp[0]);
			data.resize(0);
			data_needed.resize(0);
			double val = 0;
			double val_speed = 0;
			double val_accel = 0;

			double error = 0;
			double sum_len1 = 0;
			double sum_len2 = 0;

			cv::Vec2d pt1, pt2, pt1_0, pt2_0;

			double prev_val = 0;
			double prev_val2 = 0;
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

				pt1 = cv::Vec2d(t, val_n);
				pt2 = cv::Vec2d(t, val);

	//			if(val < 0){
	//				val = 0;
	//				val_speed = 0;
	//				val_accel = 0;
	//			}
				double sum = (val_n - val);
				error += sum * sum;

				if(i > 1){
//					double sum = val + 2 * prev_val - prev_val2;
//					error += smooth_coeff * sum * sum;
					double l1 = cv::norm(pt1 - pt1_0);
					double l2 = cv::norm(pt2 - pt2_0);

					sum_len1 += l1;
					sum_len2 += l2;
				}

				pt1_0 = pt1;
				pt2_0 = pt2;

				prev_val2 = prev_val;
				prev_val = val;
				//data.push_back(val);
				//data_needed.push_back(val_n);
			}

			error += smooth_coeff * std::abs(sum_len2 - sum_len1);

			if(error < prev_error){
#pragma omp critical
				{
					error_out = error;
					prev_error = error;

					sigma_sum = 0;
					for(size_t i = 0; i < vec_distrib.size(); i++){
						double sigma = sigmas[i]* (double)(n_samples - iter) / (n_samples);
						sigma_sum += sigma;
						vec_distrib[i] = std::uniform_real_distribution<double>(pid.coeffs[i] - sigma,
																		pid.coeffs[i] + sigma);
					}
					for(size_t i = 0; i < coeffs_output.size(); i++){
						coeffs_output[i] = pid.coeffs[i];
					}
				}
			}
//			cout << "iteration: " << iter << " error: " << error << "\n";
//			stringstream ss;
//			for(size_t i = 0; i < automatic_control::pidregulator<double>::n_coeffs; i++){
//				ss << "coeff: " << i << "[" << pid.coeffs[i] << "]\n";
//			}
//			cout << "---------\n";

#pragma omp critical
			{
				//cout << counter << " error: " << prev_error << "; sigma_sum: " << sigma_sum << endl;
			}
			counter++;
		}
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
		double y1 = (0 - min_y)/dy * mat.rows;
		cv::line(mat, cv::Point(0, mat.rows - y1), cv::Point(mat.cols, mat.rows - y1), cv::Scalar(255), 2);
		for(int i = 1; i < 5; i++){
			double y1 = (100 * i - min_y)/dy * mat.rows;
			cv::line(mat, cv::Point(0, mat.rows - y1), cv::Point(mat.cols, mat.rows - y1), cv::Scalar(255), 1);
		}
	}


	void show_wnd()
	{
		if(mat.empty()){
			cv::namedWindow(name_wnd);

//			cv::createTrackbar("C0", name_wnd, 0, max_trackbar_val, widget::change_C0, this);
//			cv::createTrackbar("P", name_wnd, 0, max_trackbar_val, widget::change_P, this);
//			cv::createTrackbar("I", name_wnd, 0, max_trackbar_val, widget::change_I, this);
//			cv::createTrackbar("D", name_wnd, 0, max_trackbar_val, widget::change_D, this);

//			cv::setTrackbarPos("C0", name_wnd, (pid.coeffs[0] - min_value_tb) * max_trackbar_val / multiply_tb);
//			cv::setTrackbarPos("P", name_wnd, (pid.coeffs[1] - min_value_tb) * max_trackbar_val / multiply_tb);
//			cv::setTrackbarPos("I", name_wnd, (pid.coeffs[2] - min_value_tb) * max_trackbar_val / multiply_tb);
//			cv::setTrackbarPos("D", name_wnd, (pid.coeffs[3] - min_value_tb) * max_trackbar_val / multiply_tb);

			generate(fnc);
		}

		stringstream ss;
		ss << "c0[" << pid.coeffs[0] << "]; p[" << pid.coeffs[1] << "]; i[" << pid.coeffs[2]
		   << "]; d[" << pid.coeffs[3] << "]";

		cv::putText(mat, ss.str(), cv::Point(10, 15), 1, 1, cv::Scalar(255, 255, 255));
		//cv::displayStatusBar(name_wnd, ss.str());

		cv::imshow(name_wnd, mat);
	}

	static void change_C0(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeffs[0] = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_P(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeffs[1] = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_I(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeffs[2] = w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
	static void change_D(int pos, void* userdata){
		widget* w = static_cast<widget*>(userdata);
		w->pid.coeffs[3] =  w->min_value_tb + 1. * w->multiply_tb * pos / w->max_trackbar_val;
		w->generate(w->fnc);
	}
};

typedef double d3[3];
const d3 ranges[] = {
	  {0, 1.5, 1}
	, {1.5, 2, 1.5}
	, {2, 2.5, 7}
	, {2.5, 3, 2}
	, {3, 4, 5}
	, {4, 4.5, 3}
	, {4.5, 100, 4}
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
#ifndef _MSC_VER
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

#endif

	widget wg(1000);
	wg.fnc = &f_t;
	wg.pid = automatic_control::pidregulator<double>(f_t(0), 0.4, 0.7, 0, 0);

	double coeffs[] = {1, 5, 1, 10};
	double sigmas[] = {10, 50, 2, 30};
	std::vector<double> coeffs_output;
	double error;

	wg.generate(f_t, 100000, coeffs, sigmas, coeffs_output, error, 5.);

	cout << "error: " << error << "; ";
	for(size_t i = 0; i < coeffs_output.size(); i++){
		cout << "coeff " << i << " [ " << coeffs_output[i] << " ]; ";
		wg.pid.coeffs[i] = coeffs_output[i];
	}
	cout << endl;

	while(!done){
//		loop_time(pidxy);
		wg.show_wnd();
		int key = cv::waitKey(10) & 0xFF;
		if(key == 27){
			break;
		}
	}

#ifndef _MSC_VER
	pthread_join(ptr, 0);
#endif

	return 0;
}

