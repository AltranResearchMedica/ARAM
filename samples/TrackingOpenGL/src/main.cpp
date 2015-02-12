#include <ARAM/TagDetector.hpp> // Main ARAM class
#include <ARAM/tag/HammingTagMatcher.hpp> // Tag validator
#include <ARAM/ROIDetector/CannyFittingDetector.hpp> // Region of interest detection
#include <GL/glut.h>
#include <opencv2/opencv.hpp> // OpenCV data structure

#include <exception> //std::exception

// These parameters must be changed for every time another camera is used
float FOV = 32;  //Camera's field of view
float winX = 640; //Camera's width size (in pixels)
float winY = 480; //Camera's height size (in pixels)
float trackerSize = 4.8; // Tracker size

// No modification needed beyond this line.

GLint faces[6][4] = {  /* Vertex indices for the 6 faces of a cube. */
  {0, 1, 2, 3}, {3, 2, 6, 7}, {7, 6, 5, 4},
  {0, 1, 5, 4}, {5, 6, 2, 1}, {7, 4, 0, 3} };
GLfloat v[8][3];  /* Will be filled in with X,Y,Z vertexes. */

GLfloat colors[6][3] = {  /* color for each cube's face*/
  {1,0,0}, {0,1,0}, {0,0,1},
  {1, 1, 0}, {0, 1, 1}, {1, 0 ,1} };



// Those variables should not be declared as global in production lines
typedef aram::TagDetector<aram::CannyFittingDetector,aram::HammingTagMatcher> Detector;
cv::VideoCapture* cap;
cv::Mat frame;
Detector *detector;
aram::Intrinsic* intr;


void mainLoop();
void init();
GLuint matToTexture(cv::Mat&);
void drawCube();
double radToDeg(double);
double degToRad(double);
void blitCameraImage(cv::Mat&);

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	init(); // glut init + ARAM initialization
	glutMainLoop(); // draw loop is started by OpenGL
	return 0;
}

void init(void)
{
	// GLUT initialization
	{
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
		glutInitWindowSize(winX, winY);
		glutCreateWindow("ARAM Test Sample 3 - OpenGL Rendering");
		glutDisplayFunc(mainLoop);
		glutIdleFunc(mainLoop);

		/* Setup cube vertex data. */
		v[0][0] = v[1][0] = v[2][0] = v[3][0] = 0;
		v[4][0] = v[5][0] = v[6][0] = v[7][0] = 1;
		v[0][1] = v[1][1] = v[4][1] = v[5][1] = 0;
		v[2][1] = v[3][1] = v[6][1] = v[7][1] = 1;
		v[0][2] = v[3][2] = v[4][2] = v[7][2] = 1;
		v[1][2] = v[2][2] = v[5][2] = v[6][2] = 0;

		// Enabling textures to blit captured images on screen
		glEnable(GL_TEXTURE_2D);
  
		/* Use depth buffering for hidden surface elimination. */
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);
		glDepthFunc(GL_LEQUAL);
		glDepthRange(0,1);

		// Setup the virtual camera's perspective (this needs to be the most accurate possible from the real camera's perspective
		glMatrixMode(GL_PROJECTION);
		gluPerspective(FOV, winX/winY, 1.0, 1000.0);

		// Setup the camera position, orientation and angle to face the same way a tracker is detected
		// position = origin (since our objective is to get trackers' position depending on camera
		// orientation = Z axe
		// angle = -Y (y is downward and gluLookAt needs to know the vertical axe)
		glMatrixMode(GL_MODELVIEW);
		gluLookAt(0.0, 0.0, 0.0,
			0.0, 0.0, 5.0,      
			0.0, -1.0, 0.0);
	}

	// ARAM initialization
	{
		// Tag detector instanciation
		detector = new Detector();
		
		// Intrinsics parameters (changes for each camera)
		intr = new aram::Intrinsic("camera_data.xml");

		// use default video input (usually your webcam)
		cap = new cv::VideoCapture(0);
		
		// Check camera opening
		if(!cap->isOpened()) throw std::exception();
	}
}

void blitCameraImage(cv::Mat& frame)
{
	// Loading frame given by openCV into an OpenGL texture
	GLuint tex = matToTexture(frame);

	// Defining it as the texture to use
	glBindTexture(GL_TEXTURE_2D, tex);
	
	glPushMatrix();

	// Defines the distance we need to dezoom to display the image at screen
	double dezoom = (winY/2)/tan(degToRad(FOV/2));

	glTranslated(-winX/2, -winY/2, dezoom);
	glScaled(winX,winY, 1);



	// Generating a quad that will display the frame
    glBegin(GL_QUADS);
	glColor3f(1.0,1.0,1.0);
	glTexCoord2i(1, 0); 
	glVertex3f(0,0,0);
    glTexCoord2i(0, 0);
	glVertex3f(0,1,0);
    glTexCoord2i(0, 1); 
	glVertex3f(1,1,0);
	glTexCoord2i(1,1); 
	glVertex3f(1,0,0);
	glEnd();
    glPopMatrix();

	// Cleaning
	glDeleteTextures(1, &tex);
}

// mainLoop called by glut when updating the scene
void mainLoop()
{	
	// capturing the current frame from camera
	(*cap) >> frame;


	// Cleaning buffers for next scene
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	// Displaying the frame as an OpenGL background
	blitCameraImage(frame);

	// Tag detection
	detector->detect(frame);
	
	cv::Mat rot, trans;

	// Looping over every tag detected in the frame
	for(aram::iteratorTag it=detector->begin();it!=detector->end();++it)
	{

		// Extracting the extrinsic matrix for each tag, giving his size (in centimeters)
		aram::Extrinsic extr = (*it)->extrinsic(*intr, trackerSize);

		// Retreiving rotation Matrix (3x3) and translation vector (3x1)
		rot = extr.rotationMatrix();
		trans = extr.translationVector();
		
		glPushMatrix();		

		// We first need to translate our basis at the final object's place.
		glTranslated(trans.at<double>(0,0), trans.at<double>(1,0), trans.at<double>(2,0));


		// We then calculate Euler angles from the 3x3 rotation matrix
		// See http://nghiaho.com/?page_id=846 for more informations.
		// The formula gives back 3 angles, alpha, beta and gamma, respectively for X, Y and Z axis.
		double alpha, beta, gamma;
		alpha = atan2(rot.at<double>(2,1),rot.at<double>(2,2));
		beta = atan2(-rot.at<double>(2,0),sqrt(pow(rot.at<double>(2,1),2)+pow(rot.at<double>(2,2),2)));
		gamma = atan2(rot.at<double>(1,0),rot.at<double>(0,0));



		// We rotate the basis to suit the final object's angles
		// As the rotation matrix is composed by X, Y and then Z rotations
		// We must rotate the basis by Z, Y and then X
		// Careful, angles are given in radians, OpenGL takes degrees
		glRotated(radToDeg(gamma), 0,0,1);
		glRotated(radToDeg(beta), 0,1,0);
		glRotated(radToDeg(alpha), 1,0,0);
		
		// We scale our 1x1x1 cube to fit the tracker's size by scaling on the three axss
		glScalef(trackerSize, trackerSize, trackerSize);

		// We finaly translate our cube so it will be displayed as laid on the tag
		glTranslated(-0.5,-0.5,-1);

		// The basis it at the right place, just finish by drawing the cube
		drawCube();

		glPopMatrix();
	}

	// Update the buffers to show the new image
	glutSwapBuffers();
}

// Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
GLuint matToTexture(cv::Mat &mat)
{
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);
 
	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);
 
	//magFilter = GL_LINEAR;
 
	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
 
	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )


	cv::Mat transposed;
	cv::transpose(mat, transposed);
	cv::flip(transposed, transposed, 1);

	GLenum inputColourFormat = GL_BGR_EXT;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}
 
	// Create the texture
	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
	             0,                 // Pyramid level (for mip-mapping) - 0 is the top level
	             GL_RGB,            // Internal colour format to convert to
	             transposed.cols,          // Image width  i.e. 640 for Kinect in standard mode
	             transposed.rows,          // Image height i.e. 480 for Kinect in standard mode
	             0,                 // Border width in pixels (can either be 1 or 0)
	             inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
	             GL_UNSIGNED_BYTE,  // Image data type
	             transposed.ptr());        // The actual image data itself

	return textureID;
}


// Displays a 1x1x1 cube
void drawCube(void)
{
	glBegin(GL_QUADS);
	for (int i = 0; i < 6; i++) {
		glColor3fv(colors[i]);
		glVertex3fv(&v[faces[i][0]][0]);
		glVertex3fv(&v[faces[i][1]][0]);
		glVertex3fv(&v[faces[i][2]][0]);
		glVertex3fv(&v[faces[i][3]][0]);
	}
	glEnd();
}


// converts any angle from radian to degree
double radToDeg(double angle)
{
	return angle*180/3.1416;
}

// converts any angle from degree to radian
double degToRad(double angle)
{
	return angle*3.1416/180;
}
