#include "stdafx.h"
#include "common.h"
#include <opencv2/core/utils/logger.hpp>
#include <random>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define DIRECTION_ONLY_SIGN_ERR 1700
#define SQUARE_VS_RECTANGLE_ERROR_IN_LENGTH 30

enum Color {
	RED,
	GREEN,
	BLUE,
	BLACK,
	OTHER_COLOR,
	COLOR_LENGTH
};

enum Shape {
	TRIANGLE,
	TRIANGLE_UP,
	TRIANGLE_DOWN,
	RECTANGLE,
	RECTANGLE_VERTICAL,
	RECTANGLE_HORIZONTAL,
	SQUARE,
	DIAMOND,
	OCTOGON,
	CIRCLE,
	OTHER_SHAPE
};

enum Sign {
	NO_ENTRY,
	AHEAD_ONLY,
	RIGHT_TURN_ONLY,
	LEFT_TURN_ONLY,
	ROUNDABOUT,
	ROUNDABOUT_PREMARKING,
	DETOUR_SIGN,
	DETOUR_LEFT_ONLY,
	DETOUR_RIGHT_ONLY,
	YIELD,
	RIGHT_OF_WAY,
	END_RIGHT_OF_WAY,
	PEDESTRIAN_CROSSING,
	PRIORITY_TO_ONCOMING_TRAFFIC,
	RAILROAD_CROSSING_150,
	RAILROAD_CROSSING_100,
	RAILROAD_CROSSING_50,
	PARTICULARLY_DANGEROUS_CURVE_LEFT,
	PARTICULARLY_DANGEROUS_CURVE_RIGHT,
	EXTREMELY_DANGEROUS_ROAD,
	OTHER_SIGN
};

wchar_t* projectPath;

bool showImage = false;

Point topMost, rightMost, leftMost, bottomMost;
int originalHeight, originalWidth, height, width;

void ShowImagesForDebug(Mat image, std::string windowName)
{
	if (showImage)
	{
		float ratio = originalWidth * 1.0f / originalHeight;
		resize(image, image, Size((int)(500 * ratio), 500));
		imshow(windowName, image);
	}
}

Color DetectColor(Vec3b pixel)
{
	uchar redChannel = pixel[2];
	uchar greenChannel = pixel[1];
	uchar blueChannel = pixel[0];

	if (redChannel < 100 && greenChannel < 100 && blueChannel < 100)
		return BLACK;
	if (redChannel > 100 && redChannel > greenChannel * 1.5 && redChannel > blueChannel * 1.5)
		return RED;
	if (greenChannel > 100 && greenChannel > redChannel * 1.5 && greenChannel > blueChannel * 1.5)
		return GREEN;
	if (blueChannel > 100 && blueChannel > redChannel * 1.5 && blueChannel > greenChannel * 1.5)
		return BLUE;
	return OTHER_COLOR;
}

bool IsInside(Mat image, int i, int j)
{
	if (i < height && j < width && i >= 0 && j >= 0)
		return true;
	return false;
}

int ObjectPerimeter(Mat blackWhiteWithNoHoles)
{
	int perimeter = 0;

	int di[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	int dj[] = { -1,  0, 1, -1, 1, -1, 0, 1 };

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			bool perimeterPixel = false;
			if (blackWhiteWithNoHoles.at<uchar>(i, j) == 0)
			{
				for (int d = 0; d < 8; d++)
				{
					if (IsInside(blackWhiteWithNoHoles, i + di[d], j + dj[d]) && (blackWhiteWithNoHoles.at<uchar>(i + di[d], j + dj[d]) == 255))
					{
						perimeterPixel = true;
						break;
					}
				}
			}
			if (perimeterPixel)
				perimeter++;
		}
	}
	return perimeter *= PI / 4;
}

int ObjectArea(Mat blackWhiteWithNoHoles)
{
	int area = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteWithNoHoles.at<uchar>(i, j) == 0)
				area++;
		}
	}
	return area;
}

std::pair<int, int> ObjectCenterMass(Mat blackWhiteWithNoHoles)
{
	int area = ObjectArea(blackWhiteWithNoHoles);

	int centerRow = 0, centerCol = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteWithNoHoles.at<uchar>(i, j) == 0)
			{
				centerRow += i;
				centerCol += j;
			}

		}
	}
	centerRow /= area;
	centerCol /= area;
	return std::make_pair(centerRow, centerCol);
}

bool IsCircle(Mat blackWhiteWithNoHoles)
{
	int area = ObjectArea(blackWhiteWithNoHoles);
	int perimeter = ObjectPerimeter(blackWhiteWithNoHoles);

	float thinessRation = 4.0f * PI * (area * 1.0f / pow(perimeter, 2) * 1.0f);

	if (std::abs(thinessRation - 1) <= 0.05)
		return true;
	return false;
}

Color DetectSignColor(Mat image, Mat blackWhiteWithNoHoles)
{
	int colorsApparitions[COLOR_LENGTH] = { 0 };
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteWithNoHoles.at<uchar>(i, j) == 0)
				colorsApparitions[DetectColor(image.at<Vec3b>(i, j))]++;
		}
	}

	int maxColorIndex = 0;
	for (int i = 0; i < COLOR_LENGTH; i++)
	{
		if (colorsApparitions[i] > colorsApparitions[maxColorIndex])
			maxColorIndex = i;
	}
	return static_cast<Color>(maxColorIndex);
}

void ConvertImageFromMultipleRGBToSimpleRGB(Mat& image)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			switch (DetectColor(image.at<Vec3b>(i, j)))
			{
			case RED:
				image.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
				break;
			case GREEN:
				image.at<Vec3b>(i, j) = Vec3b(0, 255, 0);
				break;
			case BLUE:
				image.at<Vec3b>(i, j) = Vec3b(255, 0, 0);
				break;
			case BLACK:
				image.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
				break;
			default:
				image.at<Vec3b>(i, j) = Vec3b(255, 255, 255);
			}
		}
	}
}

Mat ConvertRGBToBlackWhiteWithNoHoles(Mat image)
{
	Mat blackWhiteWithNoHoles(height, width, CV_8UC1);
	blackWhiteWithNoHoles.setTo(255);

	for (int i = 0; i < height; i++)
	{
		int start = 0, finish = 0;
		for (int j = 0; j < width; j++)
		{
			if (j == 0)
			{
				for (int k = 0; k < width; k++)
				{
					if (image.at<Vec3b>(i, k) != image.at<Vec3b>(i, k - 1))
					{
						if (start == 0)
							start = k;
						else
							finish = k;
					}
				}
			}
			else
			{
				if (start != 0 && finish != 0)
				{
					if (j >= start && j < finish)
						blackWhiteWithNoHoles.at<uchar>(i, j) = 0;
				}
			}
		}
	}

	ShowImagesForDebug(blackWhiteWithNoHoles, "Black white image with NO holes");
	return blackWhiteWithNoHoles;
}

Mat ConvertRGBToBlackWhiteWithHoles(Mat image)
{
	Mat blackWhiteWithNoHoles = ConvertRGBToBlackWhiteWithNoHoles(image);

	Mat blackWhiteImage(height, width, CV_8UC1);
	blackWhiteImage.setTo(255);

	Color signColor = DetectSignColor(image, blackWhiteWithNoHoles);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (DetectColor(image.at<Vec3b>(i, j)) == signColor) {
				blackWhiteImage.at<uchar>(i, j) = 0;
			}
		}
	}
	ShowImagesForDebug(blackWhiteImage, "Black white image with holes");
	return blackWhiteImage;
}

float getCoreCoefficient(float* core, int coreSize)
{
	float coefficient = 0.0f;
	for (int i = 0; i < coreSize; i++)
		coefficient += core[i];
	return coefficient == 0.0f ? 1.0f : 1.0f / coefficient;
}

float* GaussianFilterCore(int kernelSize, float sigma)
{
	float* core = (float*)malloc(kernelSize * kernelSize * sizeof(float));
	int positionInCore = 0;
	for (int i = 0; i < kernelSize; i++)
	{
		for (int j = 0; j < kernelSize; j++)
		{
			core[positionInCore++] = 1.0f / (2 * CV_PI * pow(sigma, 2)) * exp(-(pow(i - kernelSize / 2, 2) + pow(j - kernelSize / 2, 2)) / (2 * pow(sigma, 2)));
		}
	}
	return core;
}

Mat GaussianFilterBidimensional(Mat image, int kernelSize, float sigma)
{
	int height = image.rows;
	int width = image.cols;

	float* core = GaussianFilterCore(kernelSize, sigma);
	float coef = getCoreCoefficient(core, kernelSize * kernelSize);

	// Pentru aplicarea uniforma a filtrului Gaussian se copiaza in mijlocului imaginii paddedImage imaginea sursa cu primele si ultimele 2 linii si coloane replicate.
	int pad = (kernelSize - 1) / 2;
	Mat paddedImage;
	copyMakeBorder(image, paddedImage, pad, pad, pad, pad, BORDER_REPLICATE);

	Mat filteredImage(height, width, CV_8UC3);
	filteredImage.setTo(Vec3b(0, 0, 0));

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			float filteredPixelR = 0.0f, filteredPixelG = 0.0f, filteredPixelB = 0.0f;
			int positionInCore = 0;
			for (int ii = i; ii < i + kernelSize; ii++)
			{
				for (int jj = j; jj < j + kernelSize; jj++)
				{
					Vec3b pixel = paddedImage.at<Vec3b>(ii, jj);
					float intensity = pixel[0];
					float intensityG = pixel[1];
					float intensityR = pixel[2];

					filteredPixelB += intensity * core[positionInCore];
					filteredPixelG += intensityG * core[positionInCore];
					filteredPixelR += intensityR * core[positionInCore];

					positionInCore++;
				}
			}
			filteredImage.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(coef * filteredPixelB);
			filteredImage.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(coef * filteredPixelG);
			filteredImage.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(coef * filteredPixelR);
		}
	}
	free(core);
	ShowImagesForDebug(filteredImage, "Gaussian filtered image");
	return filteredImage;
}

int BFSTaggedObjects(Mat blackWhiteWithHoles)
{
	int di[] = { -1,-1,-1,0,0,1,1,1 };
	int dj[] = { -1,0,1,-1,1,-1,0,1 };

	int label = 0;

	Mat labels = Mat(height, width, CV_32SC1);
	labels.setTo(0);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteWithHoles.at<uchar>(i, j) != 0 && labels.at<int>(i, j) == 0)
			{
				label++;
				std::queue <std::pair<int, int>> q;
				labels.at<int>(i, j) = label;
				q.push(std::pair<int, int>(i, j));
				while (!q.empty())
				{
					std::pair<int, int> p = q.front();
					q.pop();

					for (int d = 0; d < 8; d++)
					{
						if (IsInside(blackWhiteWithHoles, p.first + di[d], p.second + dj[d]))
						{
							if (blackWhiteWithHoles.at<uchar>(p.first + di[d], p.second + dj[d]) != 0 && labels.at<int>(p.first + di[d], p.second + dj[d]) == 0)
							{
								labels.at<int>(p.first + di[d], p.second + dj[d]) = label;
								q.push(std::pair<int, int>(p.first + di[d], p.second + dj[d]));
							}
						}
					}
				}
			}
		}
	}
	return label - 1;
}

int BFSTaggedObjects(Mat image, Mat& coloredDetectedObjects)
{
	coloredDetectedObjects.setTo(Vec3b(255, 255, 255));

	int di[] = { -1,-1,-1,0,0,1,1,1 };
	int dj[] = { -1,0,1,-1,1,-1,0,1 };

	int label = 0;

	Mat labels = Mat(height, width, CV_32SC1);
	labels.setTo(0);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (image.at<uchar>(i, j) == 0 && labels.at<int>(i, j) == 0)
			{
				label++;
				std::queue <std::pair<int, int>> q;
				labels.at<int>(i, j) = label;
				q.push(std::pair<int, int>(i, j));
				while (!q.empty())
				{
					std::pair<int, int> p = q.front();
					q.pop();

					for (int d = 0; d < 8; d++)
					{
						if (IsInside(image, p.first + di[d], p.second + dj[d]))
						{
							if (image.at<uchar>(p.first + di[d], p.second + dj[d]) == 0 && labels.at<int>(p.first + di[d], p.second + dj[d]) == 0)
							{
								labels.at<int>(p.first + di[d], p.second + dj[d]) = label;
								q.push(std::pair<int, int>(p.first + di[d], p.second + dj[d]));
							}
						}
					}
				}
			}
		}
	}

	std::default_random_engine eng;
	std::uniform_int_distribution<int> d(0, 255);

	std::vector<Vec3b> colors;
	for (int i = 0; i <= label; i++)
		colors.push_back(Vec3b(d(eng), d(eng), d(eng)));

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (labels.at<int>(i, j) != 0)
				coloredDetectedObjects.at<Vec3b>(i, j) = colors[labels.at<int>(i, j)];
		}
	}

	ShowImagesForDebug(coloredDetectedObjects, "Colored detected objects");
	return label;
}

// In cazul semnelor de circulatie dreptunghiulare, aceasta functie decide forma considerata mai deprate in procesul de detectie.
Shape DetectRectangleDiamondOrSquare(Mat image)
{
	// Se calculeaza proiectiile orizonale si verticale.
	int* projOriz = (int*)calloc(width, sizeof(int));
	int* projVert = (int*)calloc(height, sizeof(int));
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (image.at<uchar>(i, j) == 0)
			{
				projOriz[j]++;
				projVert[i]++;
			}
		}
	}

	// Se calculeaza lungimea si latimea obiectului prin intermediul proiectiilor.
	int lengthProjectionHorizontal = 0, lengthProjectionVertical = 0;
	for (int i = 0; i < width; i++)
		if (projOriz[i] != 0)
			lengthProjectionHorizontal++;

	for (int i = 0; i < height; i++)
		if (projVert[i] != 0)
			lengthProjectionVertical++;

	// Se determina minimul si maximul proiectiei orizontale.
	int min = INT_MAX, max = INT_MIN;
	for (int i = 0; i < width; i++)
	{
		if (projOriz[i] < min && projOriz[i] != 0)
			min = projOriz[i];
		if (projOriz[i] > max)
			max = projOriz[i];
	}

	free(projOriz);
	free(projVert);

	// Daca diferenta dintre minim si maxim este mai mica decat 30% din valoarea maxima, se considera un dreptunghi sau un patrat. Altfel, obiectul este romb.
	if (max - min <= 0.3 * max)
	{
		// Daca diferenta intre inaltime si latime este mai mica de 30%, obiectul este un patrat.
		// Atfel, daca lunigimea este mai mare decat latimea, obiectul este un dreptunghi vertical. In caz contar, este un dreptunghi orizontal.
		int difference = abs(lengthProjectionVertical - lengthProjectionHorizontal);
		int tolerance = 0.3 * min(lengthProjectionHorizontal, lengthProjectionVertical);

		if (difference <= tolerance)
			return SQUARE;
		else if (lengthProjectionVertical > lengthProjectionHorizontal)
			return RECTANGLE_VERTICAL;
		return RECTANGLE_HORIZONTAL;
	}
	return DIAMOND;
}

// Acesta functie extrage obiectul de interes din imaginea originala, pe baza conturului determinat ca fiind semnul de circulatie.
Mat ExtractSign(Mat image, Mat contourImage)
{
	Mat sign(image.rows, image.cols, CV_8UC3);
	sign.setTo(Vec3b(255, 255, 255));

	// Se cauta primul si ultimul pixel alb in fiecare rand. Pixelii intre acesti doi indici sunt copiati din image in sign.
	for (int i = 0; i < height; i++)
	{
		int start = 0, finish = 0;
		for (int j = 0; j < width; j++)
		{
			if (j == 0)
			{
				for (int k = 0; k < width; k++)
				{
					if (contourImage.at<Vec3b>(i, k) == Vec3b(255, 255, 255))
					{
						if (start == 0)
							start = k;
						else
							finish = k;
					}
				}
			}
			else
			{
				if (start != 0 && finish != 0)
					if (j >= start && j < finish)
						sign.at<Vec3b>(i, j) = image.at<Vec3b>(i, j);
			}
		}
	}
	return sign;
}

// Aceasta functie determina forma obiectului de interes, folosind imaginea originala si conturul, returnand de asemenea semnul de circulatie identificat.
Shape ProcessContour(Mat image, std::vector<Point> contour, Mat& sign)
{
	// Se gasesc punctele extreme ale conturului: cel mai de sus, de jos, din stanga si din dreapta.
	topMost = contour[0];
	rightMost = contour[0];
	leftMost = contour[0];
	bottomMost = contour[0];

	for (const auto& point : contour) {
		if (point.y < topMost.y)
			topMost = point;
		if (point.x > rightMost.x)
			rightMost = point;
		if (point.x < leftMost.x)
			leftMost = point;
		if (point.y > bottomMost.y)
			bottomMost = point;
	}

	// Aceasta functie va genera in edgesImage imaginea aferenta conturului.
	Mat edgesImage(height, width, CV_8UC3);
	edgesImage.setTo(Vec3b(0, 0, 0));
	drawContours(edgesImage, std::vector<std::vector<Point>>{contour}, -1, Scalar(255, 255, 255), 2);

	// Se obtine imaginea semnului de circulatie identificat.
	sign = ExtractSign(image, edgesImage);
	ConvertImageFromMultipleRGBToSimpleRGB(sign);

	ShowImagesForDebug(sign, "Extracted sign");
	ShowImagesForDebug(edgesImage, "Sign edges");

	// Aceasta functie este folosita pentru a apropia un contur dat cu un poligon cu un numar mai mic de varfuri, pastrand in acelasi timp forma generala a conturului.
	// Distanta maxima dintre conturul initial si poligonul aproximat, care controleaza gradul de precizie al aproximarii, este definita ca fiind 2% din perimetrul conturului
	// initial. Acesta este un poligon inchis, aspect indicat prin flag-urile setate ca fiind adevarate.
	std::vector<Point> approx;
	approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);

	if (approx.size() == 3)
		return TRIANGLE;
	if (approx.size() == 4)
		return RECTANGLE;
	if (approx.size() > 4 && approx.size() < 10) {
		approxPolyDP(contour, approx, arcLength(contour, true) * 0.01, true);
		if (approx.size() == 8)
			return OCTOGON;
		return CIRCLE;
	}
	return OTHER_SHAPE;
}

// Aceasta functie determina forma semnului de circulatie. Returneaza de asemenea si imaginea originala redimensionata si semnul identificat, intr-o imagine aditionala.
Shape DetectShapeAndSignLocation(Mat& image, Mat& sign)
{
	originalHeight = image.rows;
	originalWidth = image.cols;

	float ratio = originalWidth * 1.0f / originalHeight;

	// Imaginea este redimensionata pentru a facilita detectia contururilor.
	if (originalHeight < 500 && originalWidth < 500)
		resize(image, image, Size(originalWidth * 2, originalHeight * 2));
	else
		resize(image, image, Size((int)(1000 * ratio), 1000));

	height = image.rows;
	width = image.cols;

	Mat edges(height, width, CV_8UC3);
	edges.setTo(Vec3b(0, 0, 0));

	// Se determina muchiile din imaginea filtrata de zgomote.
	Mat blurred = GaussianFilterBidimensional(image, 5, 1.5);
	Canny(blurred, edges, 50, 150, 3);
	ShowImagesForDebug(edges, "Edges");

	// Se gasesc contururile cele mai externe dintre cele inglobate (hierarchy) din imaginea binarizata edges. 
	// Se stocheaza doar punctele de colt ale conturului pentru a reduce memoria necesara reprezentarii acestuia.
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	for (const auto& contour : contours)
	{
		double area = contourArea(contour);
		if (area < 500) continue;

		// Se obtine forma obiectului de interes, in timp ce se obtine si imaginea cu semnul de circulatie identificat.
		Shape shape = ProcessContour(image, contour, sign);
		if (shape != OTHER_SHAPE)
			return shape;
	}
	return OTHER_SHAPE;
}

float ComputeElongationAxis(Mat image)
{
	std::pair<int, int> centerMass = ObjectCenterMass(image);

	float e1 = 0, e2 = 0;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (image.at<uchar>(i, j) == 0)
			{
				e1 += 2 * (i - centerMass.first) * (j - centerMass.second);
				e2 += pow((j - centerMass.second), 2) - pow((i - centerMass.first), 2);
			}
		}
	}

	float elongationAxisAngle = atan2(e1, e2) / 2.0f;

	Mat axisImage(height, width, CV_8UC3);
	image.copyTo(axisImage);
	line(axisImage, Point(centerMass.second - 300 / 2.0f * cos(elongationAxisAngle), centerMass.first - 300 / 2.0f * sin(elongationAxisAngle)), Point(centerMass.second + 300 / 2.0f * cos(elongationAxisAngle), centerMass.first + 300 / 2.0f * sin(elongationAxisAngle)), Scalar(255, 0, 0), 2);
	ShowImagesForDebug(axisImage, "Axis");

	return elongationAxisAngle;
}

Mat ExtractInsideObjectsFromSign(Mat image, Mat blackWhiteWithNoHoles, std::vector<Vec3b> colorsToExtract)
{
	Mat insideObjects(height, width, CV_8UC1);
	insideObjects.setTo(255);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteWithNoHoles.at<uchar>(i, j) == 0)
			{
				for (int k = 0; k < colorsToExtract.size(); k++)
				{
					if ((image.at<Vec3b>(i, j) == colorsToExtract.at(k)))
						insideObjects.at<uchar>(i, j) = 0;
				}
			}
		}
	}
	ShowImagesForDebug(insideObjects, "Inside of the sign");
	return insideObjects;
}

float DetectElongationAxisAngle(Mat image, Mat blackWhiteWithNoHoles)
{
	std::vector<Vec3b> colorsToExtract;
	colorsToExtract.push_back(Vec3b(255, 255, 255));

	Mat arrowImage = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);
	return ComputeElongationAxis(arrowImage);
}

// Aceasta functie detecteaza orientarea unui semn triunghiular.
Shape DetectTriangleUpOrDown(Mat blackWhiteImageWithNoHoles)
{
	int rowMedian = ObjectCenterMass(blackWhiteImageWithNoHoles).first;
	int numberOfWhitePixelsUp = 0;
	int numberOfWhitePixelsDown = 0;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (blackWhiteImageWithNoHoles.at<uchar>(i, j) == 0)
			{
				if (i < rowMedian)
					numberOfWhitePixelsUp++;
				else
					numberOfWhitePixelsDown++;
			}
		}
	}

	if (numberOfWhitePixelsUp < numberOfWhitePixelsDown)
		return TRIANGLE_UP;
	return TRIANGLE_DOWN;
}

Sign ProcessTriangleSign(Mat image, Mat blackWhiteWithNoHoles)
{
	if (DetectTriangleUpOrDown(blackWhiteWithNoHoles) == TRIANGLE_DOWN)
		return YIELD;

	std::vector<Vec3b> colorsToExtract;
	colorsToExtract.push_back(Vec3b(0, 0, 0));

	Mat content = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);
	Mat colors(image.rows, image.cols, CV_8UC3);

	if (BFSTaggedObjects(content, colors) == 1 && IsCircle(content))
		return EXTREMELY_DANGEROUS_ROAD;
	if (BFSTaggedObjects(content, colors) == 3)
		return ROUNDABOUT_PREMARKING;
	return OTHER_SIGN;
}

Sign ProcessDiamondSign(Mat blackWhiteWithHoles)
{
	int numberOfInsideObjects = BFSTaggedObjects(blackWhiteWithHoles);
	if (numberOfInsideObjects == 1 || numberOfInsideObjects > 2)
		return RIGHT_OF_WAY;
	return END_RIGHT_OF_WAY;
}

Sign ProcessRectangleVerticalSign(Mat image, Mat blackWhiteWithNoHoles)
{
	std::vector<Vec3b>colorsToExtract;
	colorsToExtract.push_back(Vec3b(0, 0, 255));

	Mat linesFromSign = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);
	Mat color(image.rows, image.cols, CV_8UC3);

	int numberOfInsideLines = BFSTaggedObjects(linesFromSign, color);
	if (numberOfInsideLines == 1)
		return RAILROAD_CROSSING_50;
	if (numberOfInsideLines == 2)
		return RAILROAD_CROSSING_100;
	return RAILROAD_CROSSING_150;
}

Sign DetectParticularyDangerousCurveLeftOrRight(Mat image, Mat blackWhiteWithNoHoles)
{
	std::vector<Vec3b> colorsToExtract;
	colorsToExtract.push_back(Vec3b(0, 0, 255));

	Mat lines = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);

	Mat colored(height, width, CV_8UC3);
	if (BFSTaggedObjects(lines, colored) != 3)
		return OTHER_SIGN;

	std::pair<int, int> centerMass = ObjectCenterMass(blackWhiteWithNoHoles);

	int rowMedian = centerMass.first;
	int firstSignPixel = 0, lastSignPixel = 0;

	for (int i = 0; i < width; i++)
	{
		if (blackWhiteWithNoHoles.at<uchar>(rowMedian, i) == 0)
		{
			if (firstSignPixel == 0)
				firstSignPixel = i;
			else
				lastSignPixel = i;
		}
	}

	int lengthLeft = 0, lengthRight = 0;
	while (1)
	{
		lengthLeft++;
		if (lines.at<uchar>(rowMedian, firstSignPixel) == 0)
			break;
		firstSignPixel++;
	}
	while (1)
	{
		lengthRight++;
		if (lines.at<uchar>(rowMedian, lastSignPixel) == 0)
			break;
		lastSignPixel--;
	}

	if (lengthLeft > lengthRight)
		return PARTICULARLY_DANGEROUS_CURVE_RIGHT;
	return PARTICULARLY_DANGEROUS_CURVE_LEFT;
}

Sign ProcessRectangleHorizontalSign(Mat image, Mat blackWhiteWithNoHoles)
{
	Sign sign = DetectParticularyDangerousCurveLeftOrRight(image, blackWhiteWithNoHoles);
	if (sign != OTHER_SIGN)
		return sign;
	return OTHER_SIGN;
}

bool DetectPedestianCrossingSign(Mat image, Mat blackWhiteWithNoHoles)
{
	Mat edges, edgesImage;

	edges.setTo(Vec3b(0, 0, 0));
	edgesImage.setTo(Vec3b(0, 0, 0));

	std::vector<Vec3b> colorsToExtract;
	colorsToExtract.push_back(Vec3b(255, 255, 255));

	Mat insideShape = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);

	Canny(insideShape, edges, 50, 150, 3);

	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	findContours(edges, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	edgesImage = Mat::zeros(edges.size(), CV_8UC3);

	for (const auto& contour : contours) {
		double area = contourArea(contour);
		if (area < 500) continue;

		std::vector<Point> approx;
		approxPolyDP(contour, approx, arcLength(contour, true) * 0.02, true);
		if (approx.size() == 3)
			return true;
	}
	return false;
}

bool DetectPriorityToOncomingTraffic(Mat image, Mat blackWhiteWithNoHoles)
{
	std::vector<Vec3b> colorsToExtract;
	colorsToExtract.push_back(Vec3b(255, 255, 255));
	colorsToExtract.push_back(Vec3b(0, 0, 255));

	Mat arrowsImage = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colorsToExtract);

	Mat arrowsImageColored(height, width, CV_8UC3);

	int numberOfArrows = BFSTaggedObjects(arrowsImage, arrowsImageColored);
	if (numberOfArrows != 2)
		return false;

	std::vector<Vec3b> colors;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			Vec3b color = arrowsImageColored.at<Vec3b>(i, j);
			if (std::find(colors.begin(), colors.end(), color) == colors.end() && color != Vec3b(255, 255, 255)) {
				colors.push_back(color);
			}
		}
	}

	std::vector<float> elongationAxisAngleForElements;
	int elementNr = 0;
	for (Vec3b color : colors)
	{
		Mat oneElement(height, width, CV_8UC1);
		oneElement.setTo(255);

		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				if (arrowsImageColored.at<Vec3b>(i, j) == color)
					oneElement.at<uchar>(i, j) = 0;
			}
		}

		elongationAxisAngleForElements.push_back(ComputeElongationAxis(oneElement));
	}

	if (abs(elongationAxisAngleForElements.at(0)) > 1 && abs(elongationAxisAngleForElements.at(1)) > 1)
		return true;
	return false;
}

Sign ProcessSquareSign(Mat image, Mat blackWhiteWithNoHoles)
{
	std::vector<Vec3b> colors;
	colors.push_back(Vec3b(0, 0, 0));
	Mat content = ExtractInsideObjectsFromSign(image, blackWhiteWithNoHoles, colors);

	if (DetectSignColor(image, blackWhiteWithNoHoles) == BLUE)
	{
		if (DetectPedestianCrossingSign(image, blackWhiteWithNoHoles))
			return PEDESTRIAN_CROSSING;
		if (DetectPriorityToOncomingTraffic(image, blackWhiteWithNoHoles))
			return PRIORITY_TO_ONCOMING_TRAFFIC;
	}
	return OTHER_SIGN;
}

Sign DetectBlueSignWithOneArrow(Mat image, Mat blackWhiteWithNoHoles)
{
	int colMedian = ObjectCenterMass(blackWhiteWithNoHoles).second;

	std::vector<std::pair<int, int>> limits;

	for (int i = 0; i < height; i++)
	{
		int firstChange = -1, lastChange = -1;
		for (int j = 1; j < width; j++)
		{
			if (image.at<Vec3b>(i, j) != image.at<Vec3b>(i, j - 1))
			{
				if (firstChange == -1)
					firstChange = j;
				else
					lastChange = j;
			}
		}

		limits.push_back(std::make_pair(firstChange, lastChange));
	}

	int numberOfWhitePixelsOnLeft = 0;
	int numberOfWhitePixelsOnRight = 0;

	for (int i = 0; i < limits.size(); i++)
	{
		for (int j = limits[i].first; j <= colMedian; j++)
			if (limits[i].first != -1)
				if (DetectColor(image.at<Vec3b>(i, j)) == OTHER_COLOR)
					numberOfWhitePixelsOnLeft++;

		for (int j = colMedian + 1; j < limits[i].second; j++)
			if (limits[i].second != -1)
				if (DetectColor(image.at<Vec3b>(i, j)) == OTHER_COLOR)
					numberOfWhitePixelsOnRight++;
	}

	float elongationAxisAngle = DetectElongationAxisAngle(image, blackWhiteWithNoHoles);
	if (elongationAxisAngle > 1)
		return AHEAD_ONLY;
	if (numberOfWhitePixelsOnRight - numberOfWhitePixelsOnLeft > DIRECTION_ONLY_SIGN_ERR)
	{
		if (elongationAxisAngle > 0)
			return DETOUR_RIGHT_ONLY;
		return RIGHT_TURN_ONLY;
	}
	if (numberOfWhitePixelsOnLeft - numberOfWhitePixelsOnRight > DIRECTION_ONLY_SIGN_ERR)
	{
		if (elongationAxisAngle < 0)
			return DETOUR_LEFT_ONLY;
		return LEFT_TURN_ONLY;
	}
	return OTHER_SIGN;
}

Sign ProcessCircleSign(Mat image, Mat blackWhiteWithNoHoles, Mat blackWhiteWithHoles)
{
	if (DetectSignColor(image, blackWhiteWithNoHoles) == RED)
		return NO_ENTRY;
	if (DetectSignColor(image, blackWhiteWithNoHoles) == BLUE)
	{
		int numberOfInsideObjects = BFSTaggedObjects(blackWhiteWithHoles);
		if (numberOfInsideObjects == 3)
			return ROUNDABOUT;
		if (numberOfInsideObjects == 2)
			return DETOUR_SIGN;
		Sign detectedSign = DetectBlueSignWithOneArrow(image, blackWhiteWithNoHoles);
		if (detectedSign != OTHER_SIGN)
			return detectedSign;
	}
	return OTHER_SIGN;
}

void BoundedSignImage(Mat image)
{
	Point topLeft(leftMost.x, topMost.y), bottomRight(rightMost.x, bottomMost.y);
	rectangle(image, topLeft, bottomRight, Scalar(0, 255, 0), 4);
}

void ShowOriginalImage(Mat image, std::string windowName)
{
	float ratio = originalWidth * 1.0f / originalHeight;
	resize(image, image, Size((int)(500 * ratio), 500));
	imshow(windowName, image);
}

void DetectSign()
{
	char fname[MAX_PATH];
	while (openFileDlg(fname))
	{
		Mat image = imread(fname, IMREAD_COLOR);


		Mat sign(image.rows, image.cols, CV_8UC3);

		Shape detectedShape = DetectShapeAndSignLocation(image, sign);

		Mat blackWhiteWithHoles = ConvertRGBToBlackWhiteWithHoles(sign);
		Mat blackWhiteWithNoHoles = ConvertRGBToBlackWhiteWithNoHoles(sign);

		BoundedSignImage(image);
		ShowOriginalImage(image, "Original image");

		if (detectedShape == TRIANGLE)
		{
			switch (ProcessTriangleSign(sign, blackWhiteWithNoHoles))
			{
			case YIELD:
				printf("YILED SIGN\n");
				break;
			case ROUNDABOUT_PREMARKING:
				printf("ROUNDABOUT PREMARKING SIGN\n");
				break;
			case EXTREMELY_DANGEROUS_ROAD:
				printf("EXTREMELY DANGEROUS ROAD SIGN\n");
				break;
			default:
				printf("OTHER TRIANGLE SIGN\n");
			}
		}
		else if (detectedShape == RECTANGLE)
		{
			switch (DetectRectangleDiamondOrSquare(blackWhiteWithNoHoles))
			{
			case DIAMOND:
				switch (ProcessDiamondSign(blackWhiteWithHoles))
				{
				case RIGHT_OF_WAY:
					printf("RIGHT OF WAY SIGN\n");
					break;
				case END_RIGHT_OF_WAY:
					printf("END RIGHT OF WAY SIGN\n");
					break;
				default:
					printf("OTHER DIAMOND SIGN\n");
				}
				break;
			case RECTANGLE_VERTICAL:
				switch (ProcessRectangleVerticalSign(sign, blackWhiteWithNoHoles))
				{
				case RAILROAD_CROSSING_150:
					printf("RAILROAD CROSSING IN 150 METERS SIGN\n");
					break;
				case RAILROAD_CROSSING_100:
					printf("RAILROAD CROSSING IN 100 METERS SIGN\n");
					break;
				case RAILROAD_CROSSING_50:
					printf("RAILROAD CROSSING IN 50 METERS SIGN\n");
					break;
				default:
					printf("OTHER VERTICAL RECTANGLE SIGN\n");
				}
				break;
			case RECTANGLE_HORIZONTAL:
				switch (ProcessRectangleHorizontalSign(sign, blackWhiteWithNoHoles))
				{
				case PARTICULARLY_DANGEROUS_CURVE_LEFT:
					printf("PARTICULARLY DANGEROUS CURVE LEFT\n");
					break;
				case PARTICULARLY_DANGEROUS_CURVE_RIGHT:
					printf("PARTICULARLY DANGEROUS CURVE RIGHT\n");
					break;
				default:
					printf("OTHER RECTANGLE HORIZONTAL SIGN\n");
				}
				break;
			case SQUARE:
				switch (ProcessSquareSign(sign, blackWhiteWithNoHoles))
				{
				case PEDESTRIAN_CROSSING:
					printf("PEDESTRIAN CROSSING SIGN\n");
					break;
				case PRIORITY_TO_ONCOMING_TRAFFIC:
					printf("PRIORITY TO ONCOMING TRAFFIC SIGN\n");
					break;
				default:
					printf("OTHER SQUARE SIGN\n");
					break;
				}
			}
		}
		else if (detectedShape == OCTOGON)
			printf("STOP SIGN\n");
		else if (detectedShape == CIRCLE)
		{
			switch (ProcessCircleSign(sign, blackWhiteWithNoHoles, blackWhiteWithHoles))
			{
			case NO_ENTRY:
				printf("NO ENTRY SIGN\n");
				break;
			case ROUNDABOUT:
				printf("ROUNDABOUT SIGN\n");
				break;
			case LEFT_TURN_ONLY:
				printf("LEFT TURN ONLY SIGN\n");
				break;
			case RIGHT_TURN_ONLY:
				printf("RIGHT TURN ONLY SIGN\n");
				break;
			case DETOUR_LEFT_ONLY:
				printf("DETOUR LEFT ONLY SIGN\n");
				break;
			case DETOUR_RIGHT_ONLY:
				printf("DETOUR RIGHT ONLY SIGN\n");
				break;
			case AHEAD_ONLY:
				printf("AHEAD ONLY SIGN\n");
				break;
			case DETOUR_SIGN:
				printf("DETOUR SIGN\n");
				break;
			default:
				printf("OTHER CIRCLE SIGN\n");
			}
		}

		waitKey(0);
	}
}

int main()
{
	cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_FATAL);
	projectPath = _wgetcwd(0, 0);

	int option;

	do
	{
		system("cls");
		destroyAllWindows();

		printf("Select an option:\n");
		printf("\t 0 - Exit\n");
		printf("\t 1 - Show all photos\n");
		printf("\t 2 - Show result\n");
		scanf("%d", &option);

		showImage = option == 1 ? true : false;

		DetectSign();
	} while (option != 0);

	return 0;
}