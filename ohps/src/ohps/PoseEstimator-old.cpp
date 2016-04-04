#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>

using namespace std;

const double PI = 3.1415926535897;

double euclidean(cv::Point2f& a, cv::Point2f& b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

void findOptimalThreshold(cv::Mat& image, unsigned int expectedKeypoints, cv::Mat& filteredImage, vector<cv::KeyPoint>& keypoints)
{
    cv::SimpleBlobDetector::Params blobParams;
    blobParams.filterByColor = true;
    blobParams.blobColor = 255;

    blobParams.filterByArea = true;
    blobParams.minArea = 30;
    blobParams.maxArea = 80;

    blobParams.filterByCircularity = true;
    blobParams.minCircularity = 0.8;

    blobParams.filterByInertia = true;
    blobParams.minInertiaRatio = 0.5;

    blobParams.minDistBetweenBlobs = 50;

    vector<cv::KeyPoint> foundKeypoints;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(blobParams);
	detector->detect(image, foundKeypoints);

	vector<cv::Point> area;
	vector<vector<cv::Point>> contour;
	area.push_back(cv::Point2f(0, 450));
	area.push_back(cv::Point2f(image.cols - 2, 450));
	area.push_back(cv::Point2f(image.cols - 2, 750));
	area.push_back(cv::Point2f(0, 750));
	contour.push_back(area);

    for (cv::KeyPoint &k : foundKeypoints)
    {
    	if (pointPolygonTest(area, k.pt, false) > 0) {
    		keypoints.push_back(k);
    	}
    }

	cv::Mat imageWithKeypoints;
	cv::drawKeypoints(image, keypoints, imageWithKeypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	cv::drawContours(imageWithKeypoints, contour, 0, cv::Scalar(0, 255, 0), 2);

	cv::namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
	cv::imshow("Display Image", imageWithKeypoints);

	cv::waitKey(0);
}

void findPoints(string fileName, vector<cv::Point2f>& result)
{
	cv::Mat originalImage = cv::imread(fileName, cv::IMREAD_GRAYSCALE);
    if (!originalImage.data)
    {
        printf("No image data \n");
        return;
    }

    cv::Mat filteredImage;
    vector<cv::KeyPoint> keypoints;
    findOptimalThreshold(originalImage, 5, filteredImage, keypoints);

    cout << "Keypoints:" << endl;
    for (auto it = keypoints.begin(); it != keypoints.end(); it++)
    {
    	result.push_back(cv::Point2f(it->pt));
    	cout << it->pt << endl;
    }
    cout << endl;
}

void calculateDistance(vector<cv::Point2f> keypoints, cv::Mat& distances, bool greaterThan, float threshold, bool replace = false, float replaceWith = 0.0f)
{
	int from = 0;
	for (auto i1 = keypoints.begin(); i1 != keypoints.end(); i1++, from++)
	{
		int to = 0;
		for (auto i2 = keypoints.begin(); i2 != keypoints.end(); i2++, to++)
		{
			if (from == to)
			{
				distances.at<float>(from, to) = 0.0;
			}
			else {
				float distance = euclidean(*i1, *i2);

				if (greaterThan && distance > threshold)
				{
					distances.at<float>(from, to) = replace ? replaceWith : distance;
				}
				else if (!greaterThan && distance <= threshold)
				{
					distances.at<float>(from, to) = replace ? replaceWith : distance;
				}
				else
				{
					distances.at<float>(from, to) = 0.0;
				}
			}
		}
	}
}

void classifyNodes(cv::Mat& connections, vector<int>& nonLeafs, vector<int>& leafs)
{
	cv::Mat sumNodes;
	cv::reduce(connections, sumNodes, 1, CV_REDUCE_SUM);
	for (auto n = 0; n < sumNodes.rows; n++)
	{
		if (sumNodes.at<float>(n) > 1)
		{
			nonLeafs.push_back(n);
			cout << "non-leaf: " << n + 1 << endl;
		}
		else if (sumNodes.at<float>(n) == 1)
		{
			leafs.push_back(n);
			cout << "leaf: " << n + 1 << endl;
		}
		else
		{
			cout << "ignored: " << n + 1 << endl;
		}
	}
}

void reduceVector(vector<cv::Point2f>& vectorToReduce, vector<cv::Point2f>& vectorReduced, vector<int>& keys)
{
	for (unsigned int p = 0; p < keys.size(); p++)
	{
		vectorReduced.push_back(vectorToReduce[keys[p]]);
	}
}

float measureAngle(cv::Point2f a, cv::Point2f b)
{
    float sinTheta = a.y - b.y;
    float cosTheta = a.x - b.x;

    return atan2(cosTheta, sinTheta) * (180.0f / PI);
}

int main(int argc, char** argv )
{
	vector<cv::Point2f> keypoints;
	findPoints("/media/psf/Projects/opencv/data/10.png", keypoints);

	const float DISTANCE = 80.0f;

	cv::Mat distances = cv::Mat(keypoints.size(), keypoints.size(), CV_32F);
	calculateDistance(keypoints, distances, false, DISTANCE, true, 1.0f);

	cout << distances << endl;

	vector<int> nonLeafs;
	vector<int> leafs;
	classifyNodes(distances, nonLeafs, leafs);

	vector<cv::Point2f> nonLeafKeypoints;
	reduceVector(keypoints, nonLeafKeypoints, nonLeafs);

	vector<cv::Point2f> leafKeypoints;
	reduceVector(keypoints, leafKeypoints, leafs);

	cout << nonLeafKeypoints << endl;

	cv::Mat reducedDistances = cv::Mat(nonLeafs.size(), nonLeafs.size(), CV_32F);
	calculateDistance(nonLeafKeypoints, reducedDistances, true, DISTANCE, true, 1.0f);

	cout << reducedDistances << endl;

	vector<int> vertices;
	vector<int> others;
	classifyNodes(reducedDistances, others, vertices);

	cv::Point2f A = nonLeafKeypoints.at(vertices[0]);
	cv::Point2f B = nonLeafKeypoints.at(vertices[1]);
    cv::Point2f T1 = leafKeypoints.at(0);
    cv::Point2f T2 = leafKeypoints.at(1);

    double dA = euclidean(T1, A);
    double dB = euclidean(T1, B);

    cv::Point2f S = dA < dB ? A : B;
    cv::Point2f O = dA < dB ? B : A;

    float theta1 = measureAngle(T1, S);
    float theta2 = measureAngle(T2, O);

    cout << "theta1 " << theta1 << endl;
    cout << "theta2 " << theta2 << endl;

    cout << "Theta: " << (theta1 + theta2) / 2 << endl;

	return 0;
}
