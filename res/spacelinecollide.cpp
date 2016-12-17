

Point3d two_space_linear_collision_get_nearest_point(double A_0e, double B_0e, double C_0e, double D_0e, double A_0f, double B_0f, double C_0f, double D_0f,
													double A_1e, double B_1e, double C_1e, double D_1e, double A_1f, double B_1f, double C_1f, double D_1f)
{
	//直线0的A面的D,A,B,C
	double line0_flatE_d = D_0e;

	Mat line0_flatE = (Mat_<double>(1, 3) << A_0e, B_0e, C_0e);

	//直线0的B面的D,A,B,C
	double line0_flatF_d = D_0f;
	Mat line0_flatF = (Mat_<double>(1, 3) << A_0f, B_0f, C_0f);

	//直线1的A面的D,A,B,C
	double line1_flatE_d = D_1e;
	Mat line1_flatE = (Mat_<double>(1, 3) << A_1e, B_1e, C_1e);

	//直线1的B面的D,A,B,C
	double line1_flatF_d = D_1f;
	Mat line1_flatF = (Mat_<double>(1, 3) << A_1f, B_1f, C_1f);


	//两条直线0，1的direction vector（raws = 1, cols = 3）
	Mat direction_line0 = line0_flatE.cross(line0_flatF);

	Mat direction_line1 = line1_flatE.cross(line1_flatF);

	//line0的randpoint
	Point3d randPoint0(0.0, 0.0, 0.0);
	randPoint0.z = 800;
	Mat randPoint0_in = (Mat_<double>(2, 2) << line0_flatE.at<double>(0, 0), line0_flatE.at<double>(0, 1),
		line0_flatF.at<double>(0, 0), line0_flatF.at<double>(0, 1));

	Mat randPoint0_out = randPoint0_in.inv()*(Mat_<double>(2, 1) << -line0_flatE_d - randPoint0.z*line0_flatE.at<double>(0, 2),
		-line0_flatF_d - randPoint0.z*line0_flatF.at<double>(0, 2));
	randPoint0.x = randPoint0_out.at<double>(0, 0);
	randPoint0.y = randPoint0_out.at<double>(1, 0);


	//line1上randpoint
	Point3d randPoint1(0.0, 0.0, 0.0);
	randPoint1.z = 800;
	Mat randPoint1_in = (Mat_<double>(2, 2) << line1_flatE.at<double>(0, 0), line1_flatE.at<double>(0, 1),
		line1_flatF.at<double>(0, 0), line1_flatF.at<double>(0, 1));

	Mat randPoint1_out = randPoint1_in.inv()*(Mat_<double>(2, 1) << -line1_flatE_d - randPoint1.z*line1_flatE.at<double>(0, 2),
		-line1_flatF_d - randPoint1.z*line1_flatF.at<double>(0, 2));
	randPoint1.x = randPoint1_out.at<double>(0, 0);
	randPoint1.y = randPoint1_out.at<double>(1, 0);

	//Public vertical vector(rows=1,cols=3)
	Mat publicVerticalRay = direction_line0.cross(direction_line1);


	//任选点的方向向量
	Mat randRay = (Mat_<double>(3, 1) << randPoint0.x - randPoint1.x, randPoint0.y - randPoint1.y, randPoint0.z - randPoint1.z);

	Mat cross_rand_mult = publicVerticalRay*randRay;

	//公垂线的模
	double public_vertical_vector_length = sqrtf(publicVerticalRay.at<double>(0, 0)*publicVerticalRay.at<double>(0, 0)
		+ publicVerticalRay.at<double>(0, 1)*publicVerticalRay.at<double>(0, 1)
		+ publicVerticalRay.at<double>(0, 2)*publicVerticalRay.at<double>(0, 2));

	//两直线的最小距离d=|公向量*任选向量|/|公向量|
	double minDistance = std::abs(cross_rand_mult.at<double>(0, 0)) / public_vertical_vector_length;

	Point3d point_3d(0.0,0.0,0.0);

	if (minDistance<50)
	{

		Mat two_line_t_in = (Mat_<double>(3, 3) << publicVerticalRay.at<double>(0, 0), -direction_line0.at<double>(0, 0), direction_line1.at<double>(0, 0),
			publicVerticalRay.at<double>(0, 1), -direction_line0.at<double>(0, 1), direction_line1.at<double>(0, 1),
			publicVerticalRay.at<double>(0, 2), -direction_line0.at<double>(0, 2), direction_line1.at<double>(0, 2));

		Mat two_line_t = two_line_t_in.inv()*randRay;

		Point3d node0, node1;

		node0.x = two_line_t.at<double>(1, 0)*direction_line0.at<double>(0, 0) + randPoint0.x;
		node0.y = two_line_t.at<double>(1, 0)*direction_line0.at<double>(0, 1) + randPoint0.y;
		node0.z = two_line_t.at<double>(1, 0)*direction_line0.at<double>(0, 2) + randPoint0.z;

		node1.x = two_line_t.at<double>(2, 0)*direction_line1.at<double>(0, 0) + randPoint1.x;
		node1.y = two_line_t.at<double>(2, 0)*direction_line1.at<double>(0, 1) + randPoint1.y;
		node1.z = two_line_t.at<double>(2, 0)*direction_line1.at<double>(0, 2) + randPoint1.z;

		point_3d = (node0 + node1) / 2;
		
	}
	else
	{
		point_3d = Point3d(0.0,0.0,0.0);
	}

	return point_3d;

}
