#include "graph/TransformEstimatorBase.hpp"

class TransformEstimatorGD : public TransformEstimator
{
public:
    double dd;
	double da;
	double kk;
	double ka;

    geo_u::Pose2d init_pose;
    geo_u::Pose2d estimated_pose;

	geo_u::Transform2d tf2d;

	////for log
	std::vector<geo_u::Pose2d> estimation_trajectory;
	std::vector<double> cost_transition;
	////

    TransformEstimatorGD(/* args */);
    ~TransformEstimatorGD();
    bool init(Graph &g1, Graph &g2); //inspect graphs for checking if it's ready for matching
    geo_u::Transform2d estimate(); //
    void set_init_pose();
    double optimize();
    double optimize2();
	double cost_function(double tx, double ty, double th);
};

TransformEstimatorGD::TransformEstimatorGD(/* args */)
{
    dd = 0.1;
	da = 1.;
	kk = 0.1;
	ka = 0.1;
}
TransformEstimatorGD::~TransformEstimatorGD(){}

bool TransformEstimatorGD::init(Graph &g1, Graph &g2)
{
    matched_node_set.clear();
    if (g1.node_set.size() == g2.node_set.size() && g1.node_set.size() >= 3)
    {
        for (size_t i = 0; i < g1.node_set.size(); i++)
			matched_node_set.push_back(std::make_pair(g1.node_set[i], g2.node_set[i]));
        return true;
    }
    else
    {
        std::cout << "\033[31mError[TF-EST]:  num of node: \"" << g1.node_set.size() <<", "<< g2.node_set.size() << "\" \033[m" <<std::endl;
        return false;
    }
}

geo_u::Transform2d TransformEstimatorGD::estimate()
{
	set_init_pose();
	double cost = optimize();
    return tf2d;
}

void TransformEstimatorGD::set_init_pose()
{
    double x[3], y[3], x_[3], y_[3];
	for (int i = 0; i < 3; ++i)
		x[i] = matched_node_set[i].first.position3d.x; // first.position3d -> t
	for (int i = 0; i < 3; ++i)
		y[i] = matched_node_set[i].first.position3d.y;
	for (int i = 0; i < 3; ++i)
		x_[i] = matched_node_set[i].second.position3d.x; //second.position3d -> q
	for (int i = 0; i < 3; ++i)
		y_[i] = matched_node_set[i].second.position3d.y;

	cv::Mat q = (cv::Mat_<double>(3, 3) << x[0], x[1], x[2], y[0], y[1], y[2], 1, 1, 1);
	cv::Mat t = (cv::Mat_<double>(3, 3) << x_[0], x_[1], x_[2], y_[0], y_[1], y_[2], 1, 1, 1);

	cv::Mat transform_mat = t * q.inv();
	// cv::Mat another_mat = q * t.inv();
	cv::Mat pose_mat = transform_mat; //.inv();
	init_pose.x = pose_mat.at<double>(0, 2);
	init_pose.y = pose_mat.at<double>(1, 2);
	init_pose.th = atan2(pose_mat.at<double>(1, 0), pose_mat.at<double>(0, 0)) * 180 / M_PI;
	// ROS_INFO("cos, sin: (%2.2lf, %2.2lf)", pose_mat.at<double>(1, 0), pose_mat.at<double>(0, 0));
    std::cout << "init_pose: (" << init_pose.x << ", " << init_pose.y << ", " << init_pose.th << ")" << std::endl;
    // std::cout << "init_pose: (" << another_mat.at<double>(0, 2) << ", " << another_mat.at<double>(1, 2) << ", " << atan2(another_mat.at<double>(1, 0), transform_mat.at<double>(0, 0)) * 180 / M_PI << ")" << std::endl;
}

double TransformEstimatorGD::optimize()
{
	////log
	estimation_trajectory.clear();
	cost_transition.clear();
	////
	double evmin = 100000;
	estimation_trajectory.push_back(init_pose);

	double tx, ty, th, txmin, tymin, thmin;
	tx = txmin = init_pose.x;
	ty = tymin = init_pose.y;
	th = thmin = init_pose.th;

	double evold = evmin;
	double evthre = 0.00000000001; // コスト変化閾値。変化量がこれ以下なら繰り返し終了

	double ev = cost_function(tx, ty, th);
	cost_transition.push_back(ev);
	// std::cout << "init (" << tx << ", " << ty << ", " << th << ")" << std::endl;
	while (ros::ok() && abs(evold - ev) > evthre /* && ev < 100*/)
	{
		evold = ev;
		double dEtx = (cost_function(tx + dd, ty, th) - ev) / dd;
		double dEty = (cost_function(tx, ty + dd, th) - ev) / dd;
		double dEth = (cost_function(tx, ty, th + da) - ev) / da;
		printf("dE:  dEtx=%g, dEty=%g, tdEth=%g\n", dEtx, dEty, dEth);
		double dx = -kk * dEtx;
		double dy = -kk * dEty;
		double dth = -ka * dEth;
		tx += dx;
		ty += dy;
		th += dth;
		ev = cost_function(tx, ty, th);
		if (ev < evmin)
		{
			cost_transition.push_back(ev);
			evmin = ev;
			txmin = tx;
			tymin = ty;
			thmin = th;
			estimation_trajectory.push_back(geo_u::Pose2d(txmin, tymin, thmin));
		}
	}
	estimated_pose = geo_u::Pose2d(txmin, tymin, thmin); //user pose in robot frame
    std::cout << "estimated_pose: (" << estimated_pose.x << ", " << estimated_pose.y << ", " << estimated_pose.th << ")" << std::endl;
	estimation_trajectory.push_back(estimated_pose);

	residual_error =  evmin/100;
	// transform_mat = (cv::Mat_<double>(3, 3) << cos(DEG2RAD(estimated_pose.th)), -sin(DEG2RAD(estimated_pose.th)), estimated_pose.tx, sin(DEG2RAD(estimated_pose.th)), cos(DEG2RAD(estimated_pose.th)), estimated_pose.ty, 0, 0, 1);
	tf2d = geo_u::Transform2d(estimated_pose); //user = p, robot = o
	return (evmin);
}

double TransformEstimatorGD::optimize2()
{
	////log
	estimation_trajectory.clear();
	cost_transition.clear();
	////
	double evmin = 100000;
	estimation_trajectory.push_back(init_pose);

	double tx, ty, th, txmin, tymin, thmin;
	tx = txmin = init_pose.x;
	ty = tymin = init_pose.y;
	th = thmin = init_pose.th;

	double evold = evmin;
	double evthre = 0.00000000001; // コスト変化閾値。変化量がこれ以下なら繰り返し終了

	double ev = cost_function(tx, ty, th);
	cost_transition.push_back(ev);
	// std::cout << "init (" << tx << ", " << ty << ", " << th << ")" << std::endl;
	while (ros::ok() && abs(evold - ev) > evthre /* && ev < 100*/)
	{
		evold = ev;
		double dEtx = (cost_function(tx + dd, ty, th) - ev) / dd;
		double dEty = (cost_function(tx, ty + dd, th) - ev) / dd;
		double dEth = (cost_function(tx, ty, th + da) - ev) / da;
		// printf("dE:  dEtx=%g, dEty=%g, tdEth=%g\n", dEtx, dEty, dEth);
		double dx = -kk * dEtx;
		double dy = -kk * dEty;
		double dth = -ka * dEth;
		tx += dx;
		ty += dy;
		th += dth;
		ev = cost_function(tx, ty, th);
		if (ev < evmin)
		{
			cost_transition.push_back(ev);
			evmin = ev;
			txmin = tx;
			tymin = ty;
			thmin = th;
			estimation_trajectory.push_back(geo_u::Pose2d(txmin, tymin, thmin));
		}
	}
	double best_angle = 0.;
	double min_cost = cost_function(tx, ty, 0.);
	for (size_t i = 1; i < 360; i++)
	{
		double cost = cost_function(tx, ty, double(i));
		if (cost < min_cost)
		{
			best_angle = double(i);
			min_cost = cost;
		}
	}
	thmin = best_angle;
	estimated_pose = geo_u::Pose2d(txmin, tymin, thmin); //user pose in robot frame
	std::cout << "estimated_pose: (" << estimated_pose.x << ", " << estimated_pose.y << ", " << estimated_pose.th << ")" << std::endl;
	estimation_trajectory.push_back(estimated_pose);

	residual_error =  evmin/100;
	// transform_mat = (cv::Mat_<double>(3, 3) << cos(DEG2RAD(estimated_pose.th)), -sin(DEG2RAD(estimated_pose.th)), estimated_pose.tx, sin(DEG2RAD(estimated_pose.th)), cos(DEG2RAD(estimated_pose.th)), estimated_pose.ty, 0, 0, 1);
	tf2d = geo_u::Transform2d(estimated_pose); //user = p, robot = o
	return (evmin);
}

double TransformEstimatorGD::cost_function(double tx, double ty, double th)
{
	double a = DEG2RAD(th);
	double error = 0;
	int nn = 0;
	for (size_t i = 0; i < matched_node_set.size(); ++i)
	{
		cv::Point3d q_point = matched_node_set[i].first.position3d;
		cv::Point3d t_point = matched_node_set[i].second.position3d;
		double cx = t_point.x;
		double cy = t_point.y;
		double x = cos(a) * cx - sin(a) * cy + tx;
		double y = sin(a) * cx + cos(a) * cy + ty;
		// double square_distance_error = (x - q_point.x) * (x - q_point.x) + (y - q_point.y) * (y - q_point.y);
		double square_distance_error = geo_u::square_Distance3d(cv::Point3d(x, y, 0.), q_point);
		error += square_distance_error;
		nn++;
	}
	if (nn > 0)
		error = error / nn;
	else
		error = 100000;
	return error*100;
}

