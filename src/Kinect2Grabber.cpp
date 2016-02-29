#include "Kinect2Grabber.h"
#include "Eigen/Core"
#include <iostream>
#include <fstream>

namespace mtec {
	// define the face frame features required to be computed by this application
	static const DWORD c_FaceFrameFeatures =
		/*FaceFrameFeatures::FaceFrameFeatures_None*/
		/*FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| */FaceFrameFeatures::FaceFrameFeatures_PointsInInfraredSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		/*		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement*/;

	// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
	// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
	inline void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, double* pPitch, double* pYaw, double* pRoll)
	{
		double x = pQuaternion->x;
		double y = pQuaternion->y;
		double z = pQuaternion->z;
		double w = pQuaternion->w;

		// convert face rotation quaternion to Euler angles in degrees
		*pPitch = std::atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0;
		*pYaw = std::asin(2 * (w * y - x * z)) / M_PI * 180.0;
		*pRoll = std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0;
	}

	inline void ExtractFaceRotationAsMatrix(const Vector4* pQuaternion, Eigen::Matrix3d& rotMat)
	{
		double x = pQuaternion->x;
		double y = pQuaternion->y;
		double z = pQuaternion->z;
		double w = pQuaternion->w;

		double xx = x * x;
		double xy = x * y;
		double xz = x * z;
		double xw = x * w;

		double yy = y * y;
		double yz = y * z;
		double yw = y * w;

		double zz = z * z;
		double zw = z * w;

		rotMat(0, 0) = 1 - 2 * (yy + zz);
		rotMat(0, 1) = 2 * (xy - zw);
		rotMat(0, 2) = 2 * (xz + yw);

		rotMat(1, 0) = 2 * (xy + zw);
		rotMat(1, 1) = 1 - 2 * (xx + zz);
		rotMat(1, 2) = 2 * (yz - xw);

		rotMat(2, 0) = 2 * (xz - yw);
		rotMat(2, 1) = 2 * (yz + xw);
		rotMat(2, 2) = 1 - 2 * (xx + yy);
	}

	float Kinect2Grabber::scale(float unscaledValue) {
		return scalingFactor * unscaledValue;
	}

	Kinect2Grabber::Kinect2Grabber(Streams selStream)
		: m_sensor(nullptr)
		, m_mapper(nullptr)
		, m_multiSourceReader(nullptr)
		, m_result(S_OK)
		, m_colorWidth(1920)
		, m_colorHeight(1080)
		, m_colorBuffer()
		, m_depthWidth(512)
		, m_depthHeight(424)
		, m_depthBuffer()
		, m_infraredWidth(512)
		, m_infraredHeight(424)
		, m_infraredBuffer()
		, m_facePointNormalPtr()
		, m_HDFacePointNormalPtr()
		, m_faceFound(false)
		, m_HDFaceFound(false)
		, m_running(false)
		, m_quit(false)
		, m_signalPointXYZ(nullptr)
		, nVertices(0)
		, frames(0)
		, xShift(320)
		, yShift(240)
		, scalingFactor(1000.0f)
	{
		m_facePointNormalPtr = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		m_facePointNormalPtr->resize(1);
		m_HDFacePointNormalPtr = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
		m_HDFacePointNormalPtr->resize(1);
		for (int i = 0; i < BODY_COUNT; i++)
		{
			m_pFaceFrameSources[i] = nullptr;
			m_pFaceFrameReaders[i] = nullptr;
			m_pHDFaceFrameSources[i] = nullptr;
			m_pHDFaceFrameReaders[i] = nullptr;
			m_pFaceModel[i] = nullptr;
			m_pFaceModelBuilder[i] = nullptr;
			m_pFaceAlignment[i] = nullptr;
			produce[i] = { false };
			for (int j = 0; j < FaceShapeDeformations::FaceShapeDeformations_Count; j++) {
				deformations[i][j] = 0.0f;
			}
		}
		m_selectedStream = selStream;

		// Create Sensor Instance
		m_result = GetDefaultKinectSensor(&m_sensor);
		if (FAILED(m_result)){
			throw std::exception("Exception : GetDefaultKinectSensor()");
		}

		// Open Sensor
		m_result = m_sensor->Open();
		if (FAILED(m_result)){
			throw std::exception("Exception : IKinectSensor::Open()");
		}

		// Retrieved Coordinate Mapper
		m_result = m_sensor->get_CoordinateMapper(&m_mapper);
		if (FAILED(m_result)){
			throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
		}
		
		if (m_selectedStream == DepthFace || m_selectedStream == HDFace) {

			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(m_result))
				{
					// create the face frame source by specifying the required face frame features
					if (m_selectedStream == DepthFace){
						m_result = CreateFaceFrameSource(m_sensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
						if (SUCCEEDED(m_result)) {
							// open the corresponding reader
							m_result = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
						}
					}

					// create the HD face frame source
					if (m_selectedStream == HDFace) {
						m_result = CreateHighDefinitionFaceFrameSource(m_sensor, &m_pHDFaceFrameSources[i]);
						if (SUCCEEDED(m_result)) {
							//open the corresponding reader
							m_result = m_pHDFaceFrameSources[i]->OpenReader(&m_pHDFaceFrameReaders[i]);
							if (FAILED(m_result)) {
								throw std::exception("Exception: Open HDFaceFrameReader");
							}
							//open the face model builder
							m_result = m_pHDFaceFrameSources[i]->OpenModelBuilder(FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &m_pFaceModelBuilder[i]);

							//start collecting face data
							if (SUCCEEDED(m_result)) {
								m_result = m_pFaceModelBuilder[i]->BeginFaceDataCollection();
								if (FAILED(m_result)) {
									throw std::exception("Exception: Begin Face Data Collection");
								}
							}
						}
						// Create Face Alignment
						m_result = CreateFaceAlignment(&m_pFaceAlignment[i]);
						if (FAILED(m_result)) {
							throw std::exception("Exception: CreateFaceAlignment()");
						}
						// Create Face Model
						m_result = CreateFaceModel(1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &deformations[i][0], &m_pFaceModel[i]);
						if (FAILED(m_result)) {
							throw std::exception("Exception: CreateFaceModel()");
						}
					}
				}
			}
		}

		// create the forest estimator
		if (m_selectedStream == ForestFace) {
			// load configuration and camera intrinsics
			estimator.loadConfig("config.txt");
			//load forest
			if (!estimator.loadForest(estimator.g_treepath.c_str(), estimator.g_ntrees)) {
				std::cerr << "could not read forest!" << std::endl;
				exit(-1);
			}
			img3D.create(480, 640, CV_32FC3);

			// Get Intrinsic Values
			CameraIntrinsics camInts;
			m_mapper->GetDepthCameraIntrinsics(&camInts);
			Hfl = camInts.FocalLengthX;
			Vfl = camInts.FocalLengthY;

			std::cout << "Horizontal Focal Length: " << Hfl << std::endl;
			std::cout << "Vertical Focal Length: " << Vfl << std::endl;
		}

		// Get Vertex Count
		m_result = GetFaceModelVertexCount(&nVertices); //1347
		if (FAILED(m_result)) {
			throw std::exception("Exception: GetFaceModelVertexCount()");
		}

		// To Reserve Depth Frame Buffer
		m_depthBuffer.resize(m_depthWidth * m_depthHeight);

		m_signalPointXYZ = createSignal<slotKinect2PointXYZ>();
		switch (m_selectedStream) {
		case DepthFace:
		case HDFace:
		case ForestFace:
			m_signalPointXYZFaceNormal = createSignal<slotKinect2PointXYZFaceNormal>();
			break;
		default:
			m_signalPointXYZ = createSignal<slotKinect2PointXYZ>();
		}
	}

	Kinect2Grabber::~Kinect2Grabber() throw()
	{
		stop();

		disconnect_all_slots<slotKinect2PointXYZ>();
		disconnect_all_slots<slotKinect2PointXYZFaceNormal>();

		// done with face sources and readers
		for (int i = 0; i < BODY_COUNT; i++)
		{
			SafeRelease(m_pFaceFrameSources[i]);
			SafeRelease(m_pFaceFrameReaders[i]);
			SafeRelease(m_pHDFaceFrameSources[i]);
			SafeRelease(m_pHDFaceFrameReaders[i]);
		}

		// End Processing
		if (m_sensor){
			m_sensor->Close();
		}
		SafeRelease(m_sensor);
		SafeRelease(m_mapper);
		SafeRelease(m_multiSourceReader);

		m_thread.join();
	}

	void Kinect2Grabber::start()
	{

		// Open IMultiSourceFrameReader
		switch (m_selectedStream) {
		case Depth:
		case ForestFace:
			m_result = m_sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_multiSourceReader);
			break;
		case DepthFace:
		case HDFace:
			m_result = m_sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Body,
				&m_multiSourceReader);
			break;
		default:
			m_result = m_sensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_Infrared,
				&m_multiSourceReader);
		}

		if (FAILED(m_result)){
			throw std::exception("Exception : IKinectSensor::OpenMultiSourceFrameReader()");
		}

		m_running = true;

		m_thread = boost::thread(&Kinect2Grabber::threadFunction, this);
	}

	void Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(m_mutex);

		m_quit = true;
		m_running = false;

		lock.unlock();
	}

	bool Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(m_mutex);

		return m_running;

		lock.unlock();
	}

	std::string Kinect2Grabber::getName() const{
		return std::string("Kinect2Grabber");
	}

	float Kinect2Grabber::getFramesPerSecond() const {
		return 30.0f;
	}

	void Kinect2Grabber::threadFunction() {
		while (!m_quit){
			IMultiSourceFrame* multiSourceFrame = nullptr;
			IColorFrame* colorFrame = nullptr;
			IDepthFrame* depthFrame = nullptr;
			IInfraredFrame* infraredFrame = nullptr;
			boost::unique_lock<boost::mutex> lock(m_mutex);

			// Acquire Latest Frames
			m_result = m_multiSourceReader->AcquireLatestFrame(&multiSourceFrame);

			if (SUCCEEDED(m_result)) {
				IDepthFrameReference* depthFrameReference = NULL;
				m_result = multiSourceFrame->get_DepthFrameReference(&depthFrameReference);
				if (SUCCEEDED(m_result)) {
					m_result = depthFrameReference->AcquireFrame(&depthFrame);
					if (SUCCEEDED(m_result)){
						// Retrieved Depth Data
						m_result = depthFrame->CopyFrameDataToArray(m_depthBuffer.size(), &m_depthBuffer[0]);
						if (FAILED(m_result)){
							throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
						}
						//else {
						//	frames++;
						//	std::cout << frames << std::endl;
						//}
					}
				}
				SafeRelease(depthFrameReference);
			}

			if (m_selectedStream == DepthFace || m_selectedStream == HDFace) {
				if (SUCCEEDED(m_result)) {
					IBodyFrameReference* bodyFrameReference = NULL;
					m_result = multiSourceFrame->get_BodyFrameReference(&bodyFrameReference);
					if (SUCCEEDED(m_result)) {
						IBodyFrame* pBodyFrame = nullptr;
						m_result = bodyFrameReference->AcquireFrame(&pBodyFrame);
						if (SUCCEEDED(m_result)){
							std::cout << "  IBodyFrame acquisition successful." << std::endl;
							IBody* pBody[BODY_COUNT] = { 0 };
							m_result = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
							if (SUCCEEDED(m_result)){
								std::cout << "    IBody data retrieved from the body frame successfully." << std::endl;
								for (int count = 0; count < BODY_COUNT; count++){
									BOOLEAN bTrackingIdValid = false;
									if (m_selectedStream == HDFace) {
										m_result = m_pHDFaceFrameSources[count]->get_IsTrackingIdValid(&bTrackingIdValid);
									}
									else {
										m_result = m_pFaceFrameSources[count]->get_IsTrackingIdValid(&bTrackingIdValid);
									}
									if (!bTrackingIdValid) {
										BOOLEAN bTracked = false;
										m_result = pBody[count]->get_IsTracked(&bTracked);
										if (SUCCEEDED(m_result) && bTracked){
											std::cout << "      IBody " << count << " is successfully tracked!" << std::endl;

											// Set TrackingID to Detect Face
											UINT64 trackingId = _UI64_MAX;
											m_result = pBody[count]->get_TrackingId(&trackingId);
											if (SUCCEEDED(m_result)){
												std::cout << "        IBody " << count << " has tracking ID: " << trackingId << std::endl;
												if (m_selectedStream == DepthFace){
													m_pFaceFrameSources[count]->put_TrackingId(trackingId);
												}
												if (m_selectedStream == HDFace){
													m_pHDFaceFrameSources[count]->put_TrackingId(trackingId);
												}
											}
										}
									}
								}
							}
							for (int count = 0; count < BODY_COUNT; count++){
								SafeRelease(pBody[count]);
							}
						}


						// Face Frame
						//std::system("cls");
						if (m_selectedStream == DepthFace) {
							for (int count = 0; count < BODY_COUNT; count++){
								IFaceFrame* pFaceFrame = nullptr;
								m_result = m_pFaceFrameReaders[count]->AcquireLatestFrame(&pFaceFrame);
								if (SUCCEEDED(m_result) && pFaceFrame != nullptr){
									std::cout << "  IFaceFrame acquired successfully for body " << count << std::endl;
									BOOLEAN bFaceTracked = false;
									m_result = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
									if (SUCCEEDED(m_result) && bFaceTracked){
										std::cout << "    IFaceFrame has a valid tracking ID." << std::endl;
										IFaceFrameResult* pFaceResult = nullptr;
										m_result = pFaceFrame->get_FaceFrameResult(&pFaceResult);
										if (SUCCEEDED(m_result) && pFaceResult != nullptr){
											std::cout << "    IFaceFrameResult retrieved successfully." << std::endl;

											std::vector<std::string> result;

											// Face Point
											PointF facePoint[FacePointType::FacePointType_Count];
											m_result = pFaceResult->GetFacePointsInInfraredSpace(FacePointType::FacePointType_Count, facePoint);
											if (SUCCEEDED(m_result)){
												DepthSpacePoint dsp;
												dsp.X = facePoint[FacePointType_Nose].X;
												dsp.Y = facePoint[FacePointType_Nose].Y;
												UINT16 currDepth;
												int x = static_cast<int>(dsp.X + 0.5);
												int y = static_cast<int>(dsp.Y + 0.5);
												currDepth = m_depthBuffer[m_infraredWidth*y + x];
												CameraSpacePoint csp = { 0 };
												m_result = m_mapper->MapDepthPointToCameraSpace(dsp, currDepth, &csp);
												m_facePointNormalPtr->at(0).x = scale(csp.X);
												m_facePointNormalPtr->at(0).y = scale(csp.Y);
												m_facePointNormalPtr->at(0).z = scale(csp.Z);
											}

											// Face Rotation
											Vector4 faceRotation;
											m_result = pFaceResult->get_FaceRotationQuaternion(&faceRotation);
											if (SUCCEEDED(m_result)){
												Eigen::Matrix3d rotMat;
												ExtractFaceRotationAsMatrix(&faceRotation, rotMat);
												m_facePointNormalPtr->at(0).normal_x = scale(rotMat(0, 2));
												m_facePointNormalPtr->at(0).normal_y = scale(rotMat(1, 2));
												m_facePointNormalPtr->at(0).normal_z = scale(rotMat(2, 2));
												m_faceFound = true;
											}
										}
										SafeRelease(pFaceResult);
									}
								}
								SafeRelease(pFaceFrame);
							}
						}

						//HD Face Frame
						if (m_selectedStream == HDFace) {
							for (int count = 0; count < BODY_COUNT; count++){
								IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
								m_result = m_pHDFaceFrameReaders[count]->AcquireLatestFrame(&pHDFaceFrame);
								if (SUCCEEDED(m_result) && pHDFaceFrame != nullptr){
									std::cout << "  IHDFaceFrame acquired successfully for body " << count << std::endl;
									BOOLEAN bFaceTracked = false;
									m_result = pHDFaceFrame->get_IsFaceTracked(&bFaceTracked);
									if (SUCCEEDED(m_result) && bFaceTracked){
										std::cout << "    IHDFaceFrame has a valid tracking ID." << std::endl;
										m_result = pHDFaceFrame->GetAndRefreshFaceAlignmentResult(m_pFaceAlignment[count]);
										if (SUCCEEDED(m_result) && m_pFaceAlignment[count] != nullptr) {
											std::cout << "       Face aligned successfully." << std::endl;
											// Face Model Building
											/*
											if (!produce[count]) {
											std::cout << "          Not produced yet." << std::endl;
											FaceModelBuilderCollectionStatus collection;
											m_result = m_pFaceModelBuilder[count]->get_CollectionStatus(&collection);
											if (collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete) {
											std::cout << "             Status: Complete" << std::endl;
											IFaceModelData* pFaceModelData = nullptr;
											m_result = m_pFaceModelBuilder[count]->GetFaceData(&pFaceModelData);
											if (SUCCEEDED(m_result) && pFaceModelData != nullptr) {
											std::cout << "                Face Model Data " << count << " complete!" << std::endl;
											m_result = pFaceModelData->ProduceFaceModel(&m_pFaceModel[count]);
											if (SUCCEEDED(m_result) && m_pFaceModel[count] != nullptr) {
											std::cout << "                   Face " << count << "produced!" << std::endl;
											produce[count] = true;
											}
											}
											SafeRelease(pFaceModelData);
											}
											else {
											std::cout << "Status : " << collection << std::endl;

											// Collection Status
											if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded){
											std::cout << "Need : Tilted Up Views" << std::endl;
											}
											else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded){
											std::cout << "Need : Right Views" << std::endl;
											}
											else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded){
											std::cout << "Need : Left Views" << std::endl;
											}
											else if (collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded){
											std::cout << "Need : Front ViewFrames" << std::endl;
											}

											// Capture Status
											FaceModelBuilderCaptureStatus capture;
											m_result = m_pFaceModelBuilder[count]->get_CaptureStatus(&capture);
											switch (capture){
											case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
											std::cout << "Error : Face Too Far from Camera" << std::endl;
											break;
											case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
											std::cout << "Error : Face Too Near to Camera" << std::endl;
											break;
											case FaceModelBuilderCaptureStatus_MovingTooFast:
											std::cout << "Error : Moving Too Fast" << std::endl;
											break;
											default:
											break;
											}
											}
											}
											*/
											// HD Face Point
											std::vector<CameraSpacePoint> facePoints(nVertices);
											m_result = m_pFaceModel[count]->CalculateVerticesForAlignment(m_pFaceAlignment[count], nVertices, &facePoints[0]);
											if (SUCCEEDED(m_result)) {
												std::cout << "          Vertices calculated for Face " << count << std::endl;
												CameraSpacePoint csp = { 0 };
												csp.X = facePoints[HighDetailFacePoints::HighDetailFacePoints_NoseTip].X;
												csp.Y = facePoints[HighDetailFacePoints::HighDetailFacePoints_NoseTip].Y;
												csp.Z = facePoints[HighDetailFacePoints::HighDetailFacePoints_NoseTip].Z;
												m_HDFacePointNormalPtr->at(0).x = scale(csp.X);
												m_HDFacePointNormalPtr->at(0).y = scale(csp.Y);
												m_HDFacePointNormalPtr->at(0).z = scale(csp.Z);
												//Face Rotation
												Vector4 faceRotation;
												m_result = m_pFaceAlignment[count]->get_FaceOrientation(&faceRotation);
												if SUCCEEDED(m_result){
													std::cout << "             Got Face Orientation for Face " << count << std::endl;
													Eigen::Matrix3d rotMat;
													ExtractFaceRotationAsMatrix(&faceRotation, rotMat);
													m_HDFacePointNormalPtr->at(0).normal_x = scale(rotMat(0, 2));
													m_HDFacePointNormalPtr->at(0).normal_y = scale(rotMat(1, 2));
													m_HDFacePointNormalPtr->at(0).normal_z = scale(rotMat(2, 2));
													m_HDFaceFound = true;
												}
											}
										}
									}
								}
								SafeRelease(pHDFaceFrame);
							}
						}
						SafeRelease(pBodyFrame);
					}
					SafeRelease(bodyFrameReference);
				}
			}

			if (m_selectedStream == ForestFace) {
				// head pose estimation with DRRFs, see Fanelli et al. (2011)
				// 1. get the point cloud from depth data
				for (int y = 0; y < img3D.rows; y++) {
					cv::Vec3f* img3Di = img3D.ptr<cv::Vec3f>(y);
					for (int x = 0; x < img3D.cols; x++) {
						if ((y > 26 && y < 451) && (x > 62 && x < 575)) {
							UINT16 depth = m_depthBuffer[(y - 27) * m_depthWidth + (x - 63)];
							if (depth < estimator.g_max_z && depth > 0) {
								DepthSpacePoint depthSpacePoint = { static_cast<float>(x - 63), static_cast<float>(y - 27) };
								//CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
								//m_mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
								//img3Di[x][0] = cameraSpacePoint.X;
								//img3Di[x][1] = cameraSpacePoint.Y;
								//img3Di[x][2] = cameraSpacePoint.Z;
								/*img3Di[x][0] = float(depth) * (float(x) - float(xShift)) / Hfl;
								img3Di[x][1] = float(depth) * (float(y) - float(yShift)) / Vfl;
								img3Di[x][2] = float(depth);*/
								img3Di[x][0] = float(depth) * (float(x) - xShift) / Hfl;
								img3Di[x][1] = float(depth) * (float(y) - yShift) / Vfl;
								img3Di[x][2] = float(depth);
							}
							else {
								img3Di[x] = 0;
							}
						}
						else {
							img3Di[x] = 0;
						}
					}
				}

				// 2. estimate the head pose
				g_means.clear();
				g_votes.clear();
				g_clusters.clear();
				estimator.estimate(	img3D,
									g_means,
									g_clusters,
									g_votes,
									estimator.g_stride,
									estimator.g_maxv,
									estimator.g_prob_th,
									estimator.g_larger_radius_ratio,
									estimator.g_smaller_radius_ratio,
									false,
									estimator.g_th
								);

				// 3. get pose information
				if (g_means.size() > 0) {
					//std::cout << "Face found!" << std::endl;
					m_faceFound = true;
					Eigen::Affine3f T_est = estimator.GetTransformation(g_means[0][0], g_means[0][1], g_means[0][2], g_means[0][3], g_means[0][4], g_means[0][5]);
					m_facePointNormalPtr->at(0).x = T_est(0, 3);
					m_facePointNormalPtr->at(0).y = T_est(1, 3);
					m_facePointNormalPtr->at(0).z = T_est(2, 3);
					// get normal (-z direction)
					Eigen::Vector4f faceDir_est(0.0, 0.0, -100.0, 1.0);
					faceDir_est = T_est * faceDir_est;
					m_facePointNormalPtr->at(0).normal_x = faceDir_est(0, 0);
					m_facePointNormalPtr->at(0).normal_y = faceDir_est(1, 0);
					m_facePointNormalPtr->at(0).normal_z = faceDir_est(2, 0);
					//std::cout << T_est.matrix() << std::endl;
					//std::cout << faceDir_est.matrix() << std::endl;
				}
			}

			SafeRelease(colorFrame);
			SafeRelease(depthFrame);
			SafeRelease(infraredFrame);

			lock.unlock();

			if (m_selectedStream == Depth) {
				m_signalPointXYZ->operator()(convertDepthToPointXYZ(&m_depthBuffer[0]));
			}

			if (m_selectedStream == ForestFace) {
				//m_signalPointXYZFaceNormal->operator()(convertDepthToPointXYZ(img3D), m_facePointNormalPtr, m_faceFound);
				m_signalPointXYZFaceNormal->operator()(convertDepthToPointXYZ(&m_depthBuffer[0]), m_facePointNormalPtr, m_faceFound);
			}

			if (m_selectedStream == DepthFace) {
				m_signalPointXYZFaceNormal->operator()(convertDepthToPointXYZ(&m_depthBuffer[0]), m_facePointNormalPtr, m_faceFound);
			}

			if (m_selectedStream == HDFace) {
				m_signalPointXYZFaceNormal->operator()(convertDepthToPointXYZ(&m_depthBuffer[0]), m_HDFacePointNormalPtr, m_HDFaceFound);
			}

			m_faceFound = false;
			m_HDFaceFound = false;
		}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud->width = static_cast<uint32_t>(m_depthWidth);
		cloud->height = static_cast<uint32_t>(m_depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZ* pt = &cloud->points[0];
		for (int y = 0; y < m_depthHeight; y++){
			for (int x = 0; x < m_depthWidth; x++, pt++){
				pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * m_depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint csp = { 0.0f, 0.0f, 0.0f };
				m_mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &csp);
				point.x = scale(csp.X);
				point.y = scale(csp.Y);
				point.z = scale(csp.Z);

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr Kinect2Grabber::convertDepthToPointXYZ(cv::Mat & img3D)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud->width = static_cast<uint32_t>(640);
		cloud->height = static_cast<uint32_t>(480);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZ* pt = &cloud->points[0];
		for (int y = 0; y < 480; y++){
			cv::Vec3f* img3Di = img3D.ptr<cv::Vec3f>(y);
			for (int x = 0; x < 640; x++, pt++){
				pcl::PointXYZ point;
				point.x = img3Di[x][0];
				point.y = img3Di[x][1];
				point.z = img3Di[x][2];

				*pt = point;
			}
		}

		return cloud;
	}
}
