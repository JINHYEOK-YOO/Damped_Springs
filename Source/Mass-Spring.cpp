#include "glSetup.h"

#include <Eigen/Dense>
using namespace Eigen;

#include <iostream>
#include <vector>
using namespace std;

void init();
void quit();
void initializeParticleSystem();
void update();
void solveODE();
void collisionHandling();
void keyboard(GLFWwindow* window, int key, int code, int action, int mods);
void mouse(GLFWwindow* window, int key, int action, int mods);
void render(GLFWwindow* window);
void setupLight();
void setupMaterial();
void drawSphere(float radius, const Vector3f& color, int N);
void getCursorPos(GLFWwindow* window, double* xpos, double* ypos);

// Play configuration
bool pause = false;
int frame = 0;

// Light configuration
Vector4f light(0.0, 0.0, 5.0, 1); // Light position

// Global coordinate frame
float AXIS_LENGTH = 3;
float AXIS_LINE_WIDTH = 2;

// Colors
GLfloat bgColor[4] = { 1, 1, 1, 1 };

// Sphere
GLUquadricObj*  sphere = NULL;

// Particles
vector<Vector3f>	p; // Particle position
vector<Vector3f>	v; // Particle velocity	

// Nail Constraint
vector<bool>	constrained;	// Constrained particles

// Contact
vector<bool>        contact;    // Contact state
vector<Vector3f>    contactN;   // Contact normal vector

// Selection
vector<int>	selected;			// Selected particles for connection

// Connectivity
vector<int>		e1; 						// One end of the edge
vector<int>		e2;							// The other end of the edge
vector<float>	l;							// Rest length between particles
vector<float>   k;							// Spring constants
float       	k0 = 1.0;					// Global spring constant
float			c0 = 0.01;					// Damping coefficient

// Geometry and mass
float radius = 0.02;    // 2cm
float m = 0.01;         // 10g

// Time stepping
int N_SUBSTEPS = 1;             // Sub-time steps per frame
float h = 1.0/60.0/N_SUBSTEPS;  // Time step

// External force
bool		useGravity = true;
Vector3f	gravity(0, -9.8, 0);   // Gravity -9.8m/s^2
bool		useDamping = false;

// Collision
float k_r = 0.75;               // Coefficient of restitution
float epsilon = 1.0E-4;

// Wall
const int   nWalls = 4;
Vector3f    wallP[nWalls];          // Points in the walls
Vector3f    wallN[nWalls];          // Normal vectors of the walls

// Mouse control
const float CURSOR_RANGE = 0.1;		// Particle selection range
double		xpos;
double		ypos;
bool		isDragging = false;
int			draggingParticle;

// Method
enum IntegrationMethod {
    EULER = 1,
    MODIFIED_EULER,
}   intMethod = MODIFIED_EULER;

// Mode
enum InterfaceMode {
	CREATE_DRAG = 1,
	ATTACH,
	NAIL,
}	intMode = CREATE_DRAG;

int
main(int argc, char* argv[])
{
    // Orthographics viewing
    perspective = false;
    
    // Initialize the OpenGL system
    GLFWwindow* window = initializeOpenGL(argc, argv, bgColor);
    if (window == NULL) return -1;
    
    // Vertical sync for 60fps
    glfwSwapInterval(1);
    
    // Callbacks
    glfwSetKeyCallback(window, keyboard);
	glfwSetMouseButtonCallback(window, mouse);

    // Depth test
    glEnable(GL_DEPTH_TEST);
    
    // Normal vectors are normalized after transformation.
    glEnable(GL_NORMALIZE);
    
    // Back face culling
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    
    // Viewport and perspective setting
    reshape(window, windowW, windowH);
    
    // Initialization - Main loop - Finalization
    //
    init();
   	initializeParticleSystem();
    
    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        if (!pause) update();
        
        render(window);             // Draw one frame
        glfwSwapBuffers(window);    // Swap buffers
        glfwPollEvents();           // Events

		if (isDragging)
		{
			getCursorPos(window, &xpos, &ypos);	
			p[draggingParticle][0] = xpos;
			p[draggingParticle][1] = ypos;
		}
    }

    // Finalization
    quit();
    
    // Terminate the glfw system
    glfwDestroyWindow(window); // Is it required?
    glfwTerminate();
    
    return 0;
}

void
init()
{
    // Prepare quadric shapes
    sphere = gluNewQuadric();
    gluQuadricDrawStyle(sphere, GLU_FILL);
    gluQuadricNormals(sphere, GLU_SMOOTH);
    gluQuadricOrientation(sphere, GLU_OUTSIDE);
    gluQuadricTexture(sphere, GL_FALSE);
    
    // Keyboard and mouse
    cout << "Keyboard Input: space for play on/off" << endl;
    cout << "Keyboard Input: g for gravity on/off" << endl;
    cout << "Keyboard Input: i for the system initialization" << endl;
    cout << "Keyboard Input: r for create and drag mode on" << endl;
	cout << "Keyboard Input: a for attach mode on" << endl;
	cout << "Keyboard Input: n for nail mode on" << endl;
    cout << "Keyboard Input: c for removal of all constraints" << endl;
    cout << "Keyboard Input: e for the Euler integration" << endl;
    cout << "Keyboard Input: m for the modified Euler integration" << endl;
    cout << "Keyboard Input: up to increase the spring constant" << endl;
    cout << "Keyboard Input: down to decrease the spring constant" << endl;    
	cout << "Keyboard Input: right to increase the sub-time steps" << endl;    
	cout << "Keyboard Input: left to decrease the sub-time steps" << endl;
}

void
initializeParticleSystem()
{
	// Initialize particles
	p.clear();
	v.clear();
	selected.clear();
	constrained.clear();
	contact.clear();
	contactN.clear();
	
	// Initialize connectivity
	e1.clear();
	e2.clear();
	l.clear();
	k.clear();
	
	// Default mode
	intMode = CREATE_DRAG;

    // Normal vectors of the 4 surrounding walls
    wallN[0] = Vector3f( 1.0, 0, 0);    // Left
    wallN[1] = Vector3f(-1.0, 0, 0);    // Right
    wallN[2] = Vector3f(0, 1.0, 0);     // Bottom
    wallN[3] = Vector3f(0, -1.0, 0);    // Top
    
    for (int i = 0; i < nWalls; i++)
        wallN[i].normalize();
    
    // Collision handling
    collisionHandling();
}

void
createParticle(double xpos, double ypos)
{
	p.push_back(Vector3f(xpos, ypos, 0));			// Position
	v.push_back(Vector3f::Zero());					// Velocity
	constrained.push_back(true);					// Constrained
	contact.push_back(false);						// Contact
	contactN.push_back(Vector3f::Zero());			// Contact normal
}

int
selectParticle(double xpos, double ypos)
{
	float minDist = CURSOR_RANGE;	// Minimum distance
	int nearParticle = -1;			// Selected particle

	for (int i = 0; i < p.size(); i++)
    {	
    	float dist = (Vector3f(xpos, ypos, 0) - p[i]).norm();
        if (dist < minDist)
        {
            nearParticle = i;
            minDist = dist;
        }
	}
	
	return nearParticle;
}

void
attachSpring(int index)
{
	selected.push_back(index);

	if (selected.size() >= 2)
	{
		if (selected[0] != selected[1])
		{
			// Connection
			e1.push_back(selected[0]);
			e2.push_back(selected[1]);
			l.push_back((p[e1.back()] - p[e2.back()]).norm());	// Rest length
			k.push_back(k0 / l.back());							// Spring constant
		}
		selected.clear();
	}
}

void
dragParticle(int index)
{
	draggingParticle = index;
	isDragging = true;
}

void
nailParticle(int index)
{
	constrained[index] = !constrained[index];
}

void
removeAllConst()
{
	for (int i = 0; i < constrained.size(); i++)
		constrained[i] = false;
}

void
update()
{
    // Solve ordinary differential equation
    for (int i = 0; i < N_SUBSTEPS; i++)
        solveODE();
    
    // Time increment
    frame++;
}

void
solveODE()
{
	if (p.size() > 0)
	{
    	// Total force
    	Vector3f    f[p.size()];

    	for (int i = 0; i < p.size(); i++)
    	{
        	// Initialization
        	f[i].setZero();
        
        	// Gravity
        	if (useGravity) f[i] += m * gravity;
   
		}    
    	for (int i = 0; i < l.size(); i++)
    	{
        	Vector3f    v_i = p[e1[i]] - p[e2[i]];
        	float       L_i = v_i.norm();
        	Vector3f    f_i = k[i] * (L_i - l[i]) * v_i / L_i;

        	f[e2[i]] += f_i;
        	f[e1[i]] -= f_i;
    	}
    
    	for (int i = 0; i < p.size(); i++)
    	{
        	// Constraint
        	if (constrained[i])  continue;
        
        	// Contact force
        	if (contact[i]) f[i] -= contactN[i].dot(f[i]) * contactN[i];
        
        	// Time stepping
        	switch (intMethod)
        	{
        	case EULER:
				if (useDamping)
				{
					// Spring direction
					Vector3f v_s(0, 0, 0);
					for (int j = 0; j < e1.size(); j++)
						if (e1[j] == i)	v_s += p[e2[j]] - p[e1[j]];
					for (int j = 0; j < e2.size(); j++)
						if (e2[j] == i)	v_s += p[e1[j]] - p[e2[j]];
					v_s.normalize();

            		p[i] += h * v[i];
            		v[i] += h * (f[i] - c0 * v[i].dot(v_s) * v_s) / m;
				}
				else
				{
					p[i] += h * v[i];
					v[i] += h * f[i] / m;
				}
            	break;
                
        	case MODIFIED_EULER:
				if (useDamping)
				{
					// Spring Direction
					Vector3f v_s(0, 0, 0);
					for (int j = 0; j < e1.size(); j++)
						if (e1[j] == i)	v_s += p[e2[j]] - p[e1[j]];
					for (int j = 0; j < e2.size(); j++)
						if (e2[j] == i)	v_s += p[e1[j]] - p[e2[j]];
					v_s.normalize();

           			v[i] += h * (f[i] - c0 * v[i].dot(v_s) * v_s) / m;
            		p[i] += h * v[i];
				}
				else
				{
					v[i] += h * f[i] / m;
					p[i] += h * v[i];
				}
            	break;
        	}
    	}
    	// Collision handling
    	collisionHandling();
	}
}

void
collisionHandling()
{
    // Points of the 4 surrounding walls: It can be changed.
    wallP[0] = Vector3f(-1.0 * aspect, 0, 0);   // Left
    wallP[1] = Vector3f( 1.0 * aspect, 0, 0);   // Right
    wallP[2] = Vector3f(0, -1.0, 0);            // Bottom
    wallP[3] = Vector3f(0,  1.0, 0);            // Top
    
    // Collision wrt the walls
    for (int i = 0; i < p.size(); i++)
    {
        contact[i] = false;
        
        for (int j = 0; j < nWalls; j++)
        {
            float d_N = wallN[j].dot(p[i] - wallP[j]);
            if (d_N < radius)
            {
                // Position correction
                p[i] += (radius - d_N) * wallN[j];
                
                // Normal velcoity
                float v_N = wallN[j].dot(v[i]);
                
                if (fabs(v_N) < epsilon)    // Contact check
                {
                    contact[i] = true;
                    contactN[i] = wallN[j];
                }
                else if (v_N < 0)           // Velcoity correction
                {
                    v[i] -= (1 + k_r) * v_N * wallN[j];
                }
            }
        }
    }
}

void
render(GLFWwindow* window)
{
    // Background color
    glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Axes
    if (0)
    {
        glDisable(GL_LIGHTING);
        drawAxes(AXIS_LENGTH, AXIS_LINE_WIDTH);
    }
    
    // Lighting
    setupLight();
    
    // Material
    setupMaterial();
    
    // Particles
    for (int i = 0; i < p.size(); i++)
    {
        glPushMatrix();
        glTranslatef(p[i][0], p[i][1], p[i][2]);
		if (constrained[i])	drawSphere(radius, Vector3f(1, 1, 0), 20);
        else                drawSphere(radius, Vector3f(0, 1, 0), 20);
        glPopMatrix();
    }
    
    // Edges
    glLineWidth(7);
    glColor3f(0, 0, 1);
    glBegin(GL_LINES);
    for (int i = 0; i < l.size(); i++)
    {
        glVertex3fv(p[e1[i]].data());
        glVertex3fv(p[e2[i]].data());
    }
    glEnd();
}

void
rebuildSpringK()
{
    cout << "Spring constant = " << k0 << endl;
    
    // Spring constants
    for (int i = 0; i < l.size(); i++)
        k[i] = k0 / l[i]; // Inversely proportion to the spring length
}

void
rebuildSubTimeStep()
{
	cout << "Sub-time steps = " << N_SUBSTEPS << endl;
	
	h = 1.0/60.0/N_SUBSTEPS;
}

void
quit()
{
    // Delete quadric shapes
    gluDeleteQuadric(sphere);
}

void
keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch(key)
        {
        // Quit
        case GLFW_KEY_Q:
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GL_TRUE); break;
                
        // Controls
        case GLFW_KEY_SPACE:    pause = !pause;				break;  // Play on/off
        case GLFW_KEY_G:        useGravity = !useGravity;   break;	// Gravity on/off
        case GLFW_KEY_I:        initializeParticleSystem(); break;
        case GLFW_KEY_E:        intMethod = EULER; 			break;
        case GLFW_KEY_M:        intMethod = MODIFIED_EULER; break;
		case GLFW_KEY_D:		useDamping = !useDamping;	break;	// Damping on/off

		case GLFW_KEY_R:		intMode = CREATE_DRAG;		break;	// Create and drag mode on
		case GLFW_KEY_A:		intMode = ATTACH;			break;	// Attach mode on
		case GLFW_KEY_N:		intMode = NAIL;				break;	// Nail mode on

        case GLFW_KEY_C:		removeAllConst();			break;

        // Spring constants
        case GLFW_KEY_UP:   k0 = min(k0 + 0.1, 10.0);   rebuildSpringK(); break;
        case GLFW_KEY_DOWN: k0 = max(k0 - 0.1, 0.1);    rebuildSpringK(); break;
                
        // Special keys
        case GLFW_KEY_PERIOD:           break;
        case GLFW_KEY_LEFT_BRACKET:     break;
        case GLFW_KEY_RIGHT_BRACKET:    break;

		// Sub-tiem steps
		case GLFW_KEY_RIGHT:	N_SUBSTEPS = min(N_SUBSTEPS + 1, 20);	rebuildSubTimeStep();	break;
        case GLFW_KEY_LEFT:		N_SUBSTEPS = max(N_SUBSTEPS - 1, 1);		rebuildSubTimeStep();	break;


        //default:  cerr << "key = " << (int)key << endl; break;
        }
    }
}

void
mouse(GLFWwindow* window, int button, int action, int mods)
{
	if (action == GLFW_PRESS && button == GLFW_MOUSE_BUTTON_LEFT)
	{	
		getCursorPos(window, &xpos, &ypos);
		
		int index = selectParticle(xpos, ypos);
		switch (intMode)
		{
		case CREATE_DRAG:									// Create/Drag
			if (index >= 0)	dragParticle(index);
			else			createParticle(xpos, ypos);
			break;

		case ATTACH:										// Attach
			if (index >= 0)	attachSpring(index);
			break;

		case NAIL:											// Nail
			if (index >= 0)	nailParticle(index);
			break;
		}
	}
	else if (action == GLFW_RELEASE && button == GLFW_MOUSE_BUTTON_LEFT)
    {
		isDragging = false;
	}

}

void
getCursorPos(GLFWwindow* window, double* xpos, double* ypos)
{
	// In the screen coordinate
	glfwGetCursorPos(window, xpos, ypos);
	//cout << "Mouse at (" << *xpos << ", " << *ypos << ")" << endl;

	// In the workspace
	float	aspect = (float)screenW/screenH;
	*xpos =  2.0 * (*xpos/screenW - 0.5) * aspect;
	*ypos = -2.0 * (*ypos/screenH - 0.5);
	//cout << "Particle: (" << *xpos << ", " << *ypos << ")" << endl;
}

// Light
void
setupLight()
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    
    GLfloat ambient[4] = { 0.1, 0.1, 0.1, 1 };
    GLfloat diffuse[4] = { 1.0, 1.0, 1.0, 1 };
    GLfloat specular[4] = { 1.0, 1.0, 1.0, 1 };
    
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light.data());
}

// Material
void
setupMaterial()
{
    // Material
    GLfloat mat_ambient[4] = { 0.1, 0.1, 0.1, 1 };
    GLfloat mat_specular[4] = { 0.5, 0.5, 0.5, 1 };
    GLfloat mat_shininess = 128;
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

void
setDiffuseColor(const Vector3f& color)
{
    GLfloat mat_diffuse[4] = { color[0], color[1], color[2], 1 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
}

// Draw a sphere after setting up its material
void
drawSphere(float radius, const Vector3f& color, int N)
{
    // Material
    setDiffuseColor(color);
    
    // Sphere using GLU quadrics
    gluSphere(sphere, radius, N, N);
}
