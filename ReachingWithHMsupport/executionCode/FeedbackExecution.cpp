//---------------------------------------------------------------------
//          SYSTEMID
//
// 
//---------------------------------------------------------------------

#include "stdafx.h"
#include "UDP_Test2.h"
#include "HapticAPI2.h"
#include "HapticMASTER.h"
#include <glut.h>
#include "CPrecisionTimer.h"
#include <math.h>

/*****************************************************************
 * ND Library Files Included
 *****************************************************************/
#include "ndtypes.h"
#include "ndpack.h"
#include "ndopto.h"
#include "C:\Users\KirschLab\Documents\OLD_NDIoapi(C)\samples\sleep.c"

#define IPADDRESS "192.168.0.45" // Haptic master
#define HOSTIP "192.168.0.8" // this computer
#define TARGETIP "192.168.0.2" // xpc target computer

#define PosX 0
#define PosY 1
#define PosZ 2

// Optotrak definitions
#define RIGID_OUTPUT_WIDTH 8
#define MARKER_OUTPUT_WIDTH 4
#define NUM_RIGID_BODIES 3
#define NUM_MARKERS 18
#define NUM_MARKERS_PORT1 18
#define NUM_MARKERS_PORT2 0
#define NUM_MARKERS_PORT3 0
#define NUM_MARKERS_PORT4 0
#define HM_OUTPUT_WIDTH 3
#define FREQUENCY 52
#define CAMFILE "Aligned20191217.cam"
#define THORAX_FILE "thorax_aligned.rig"
#define FIRST_THORAX 1
#define HUMERUS_FILE "humerus_aligned.rig"
#define FIRST_HUMERUS 6
#define FOREARM_FILE "forearm_aligned.rig"
#define FIRST_FOREARM 12

// HapticMaster definitions
#define HM_XOFFSET 100
#define HM_YOFFSET 0
#define HM_ZOFFSET -35

// muscle stuff
#define TARGETS 1
#define MUSCLES 10

// HapticMaster declarations
long dev = 0;
char response[100];
double CurrentPosition[3];
double CurrentPositionCyl[3];
double CurrentVelocity[3];
double CurrentAcceleration[3];
double CurrentForce[3];
double pot[3];
double inertia = 4;
double DamperCoeff[3] = {20,20,20};
double stiffness = 100;
double easyStiffness = 30;
double middleStiffness = 1000;
double hardStiffness = 2000;
double moveDamping = 0.5;
double maxSpringForce = 100;
CPrecisionTimer timer;

// count of key "s" pressed
int s_count = 0;
// UDP socket objects
CSocket UDP_Sender;
CSocket UDP_Receiver;

// Optotrak data structures
struct OptotrakRigidStruct pData6d[NUM_RIGID_BODIES];
Position3d pData3d[NUM_MARKERS];
double **dataout = NULL;
double **posout = NULL;
double **previous_dataout = NULL;
double **previous_posout = NULL;

// Optotrak declarations
//int nSensors, nRigidBodies, nMarkers;
unsigned int uFlags, uElements, uFrameNumber;
int error = 0;
int globalRigidBody = 1;
//int nIREDPositions = 9;
char szNDErrorString[MAX_ERROR_STRING_LENGTH + 1];
float samplingRate, firingRate;

// UDP declarations
double packetSize = 111;
double receivePacketSize = 4;
double udpData[111];
double udpReceiveData[4];

int writeCount = 0;
int targetNumber = 1;
double wristPos[4];
double passiveForce[3];
double target[TARGETS][3];  // [x,y,z] coordinates of the target
int muscleOrder[TARGETS][MUSCLES];
int shortMuscleOrder[MUSCLES];
int targetOrder[TARGETS];
//---------------------------------------------------------------------
// O P E N G L   M A T E R I A L S
//---------------------------------------------------------------------
// EndEffector OpenGL Material Parameters.
GLfloat EndEffectorAmbient[] = {0.91, 0.44, 0.00, 1.00};
GLfloat EndEffectorDiffuse[] = {0.90, 0.38, 0.00, 1.00};

// Block OpenGL Material Parameters.
GLfloat BlockAmbient[] = {0.00, 0.66, 0.60, 1.00};
GLfloat BlockDiffuse[] = {0.00, 0.80, 0.67, 1.00};

// General OpenGL Material Parameters
GLfloat Specular[] = {1.00, 1.00, 1.00, 1.00};
GLfloat Emissive[] = {0.00, 0.00, 0.00, 1.00};
GLfloat Shininess = {128.00};

GLfloat SpecularOff[] = {0.00, 0.00, 0.00, 0.00};
GLfloat EmissiveOff[] = {0.50, 0.50, 0.50, 0.00};
GLfloat ShininessOff = {0.00};

//---------------------------------------------------------------------
//                 O B J E C T   P A R A M E T E R S
//---------------------------------------------------------------------
double blockSize[3] = {0.15,0.15,0.15};
double blockPos[3] = {0.0,0.0,0.0};
double blockStiffness = 20000;

//---------------------------------------------------------------------
//              E N D   E F F E C T O R   M A T E R I A L
//
// EndEffectorMaterial() Sets The Current OpenGl Material Paremeters. 
// Call This Function Prior To Drawing The EndEffector.
//---------------------------------------------------------------------
void EndEffectorMaterial()
{
   glMaterialfv(GL_FRONT, GL_AMBIENT, EndEffectorAmbient);
   glMaterialfv(GL_FRONT, GL_DIFFUSE, EndEffectorDiffuse);
   glMaterialfv(GL_FRONT, GL_SPECULAR, Specular);
   glMaterialfv(GL_FRONT, GL_EMISSION, Emissive);
   glMaterialf(GL_FRONT, GL_SHININESS, Shininess);
}

//---------------------------------------------------------------------
//                     B L O C K   M A T E R I A L
//
// BlockMaterial() Sets The Current OpenGl Material Paremeters. 
// Call This Function Prior To Drawing The Block.
//---------------------------------------------------------------------
void BlockMaterial()
{
   glMaterialfv(GL_FRONT, GL_AMBIENT, BlockAmbient);
   glMaterialfv(GL_FRONT, GL_DIFFUSE, BlockDiffuse);
   glMaterialfv(GL_FRONT, GL_SPECULAR, SpecularOff);
   glMaterialfv(GL_FRONT, GL_EMISSION, Emissive);
   glMaterialf(GL_FRONT, GL_SHININESS, ShininessOff);
}

//---------------------------------------------------------------------
//                        D R A W   B L O C K 
//
// This Function Is Called To Draw The Graphic Equivalent Of 
// The Haptic Block Object In OpenGl
//---------------------------------------------------------------------
void DrawBlock(void)
{
   glPushMatrix();
   glTranslatef(blockPos[PosX], blockPos[PosY], blockPos[PosZ]);
   glScalef(blockSize[0], blockSize[1], blockSize[2]);
   BlockMaterial();
   glutSolidCube(1.0);   
   glPopMatrix();
}

//---------------------------------------------------------------------
//                  D R A W   E N D   E F F E C T O R
//
// This Function Is Called To Draw The Graphic Equivalent Of 
// The EndEffector In OpenGl.
// The EndEffector Is Drawn At The Current Position
//---------------------------------------------------------------------
void DrawEndEffector(void)
{
   EndEffectorMaterial();
   glPushMatrix();
   glTranslatef(CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ]);
   glutSolidSphere(0.005, 20, 20);
   glPopMatrix();
}

//---------------------------------------------------------------------
//                         I N I T   O P E N   G L
//
// This Function Initializes the OpenGl Graphics Engine
//---------------------------------------------------------------------
void InitOpenGl (void)
{

   glShadeModel(GL_SMOOTH);

   glLoadIdentity();
   
   GLfloat GrayLight[] = {0.75, 0.75, 0.75, 1.0};
   GLfloat LightPosition[] = {1.0, 2.0, 1.0, 0.0};
   GLfloat LightDirection[] = {0.0, 0.0, -1.0, 0.0};

   glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
   glLightfv(GL_LIGHT0, GL_AMBIENT, GrayLight);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, GrayLight);
   glLightfv(GL_LIGHT0, GL_SPECULAR, GrayLight);
   
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_DEPTH_TEST);
   glEnable(GL_NORMALIZE);
   
   glEnable (GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   
   glClearColor(0.0, 0.0, 0.3, 0.0);
}

//---------------------------------------------------------------------
//                           D I S P L A Y
//
// This Function Is Called By OpenGL To Redraw The Scene
// Here's Where You Put The EndEffector And Block Drawing FuntionCalls
//---------------------------------------------------------------------
void Display (void)
{
   glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glPushMatrix ();
    
   // define eyepoint in such a way that
   // drawing can be done as in lab-frame rather than sgi-frame
   // (so X towards user, Z is up)
   gluLookAt (1.0, 0.5, 0.5, 0.0, 0.0, -0.03, 0.0, 0.0, 1.0);
   glutPostRedisplay();

   DrawAxes();
   DrawWorkspace(dev, 3);
   DrawEndEffector();
   //DrawBlock();
   
   glPopMatrix ();
   glutSwapBuffers();
}

//---------------------------------------------------------------------
//                            R E S H A P E 
//
// The Function Is Called By OpenGL Whenever The Window Is Resized
//---------------------------------------------------------------------
void Reshape(int iWidth, int iHeight)
{
   glViewport (0, 0, (GLsizei)iWidth, (GLsizei)iHeight);
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();

   float fAspect = (float)iWidth/iHeight;
   gluPerspective (30.0, fAspect, 0.05, 20.0);            
 
   glMatrixMode (GL_MODELVIEW);
   glLoadIdentity ();
}

//---------------------------------------------------------------------
//                          Init_HM
//
// This function initializes the HapticMaster
//---------------------------------------------------------------------
void Init_HM(void)
{
	// Open the HapticMASTER device
	dev = haDeviceOpen(IPADDRESS);

	if (dev == HARET_ERROR) {
		printf("--- ERROR: Unable to connect to device: %s\n", IPADDRESS);
		//return HARET_ERROR;
	}
	else {

		InitializeDevice(dev);

		if (haSendCommand(dev, "remove all", response)) {
			printf("---ERROR: remove all: %s\n", response);
		}

		//if ( haSendCommand(dev, "set state home", response) ) {
		//      printf("---ERROR: set state home: %s\n", response);
		//}

		if (haSendCommand(dev, "set inertia", inertia, response)) {
			printf("--- ERROR: set inertia: %s\n", response);
		}

		// Create a damper effect
		if (haSendCommand(dev, "create damper myDamper", response)) {
			printf("--- ERROR: create damper myDamper: %s\n", response);
		}
		else {
			if (haSendCommand(dev, "set myDamper dampcoef", DamperCoeff[0], DamperCoeff[1], DamperCoeff[2], response)) {
				printf("--- ERROR: set myDamper dampcoef: %s\n", response);
			}
			if (haSendCommand(dev, "set myDamper enable", response)) {
				printf("--- ERROR: set myDamper enable: %s\n", response);
			}
		}

		// create a spring but do not enable it
		if (haSendCommand(dev, "create spring mySpring", response)) {
			printf("--- ERROR: create mySpring: %s\n", response);
		}
		else {
			if (haSendCommand(dev, "set mySpring stiffness", stiffness, response)) {
				printf("--- ERROR: set mySpring dampcoef: %s\n", response);
			}
			if (haSendCommand(dev, "set mySpring maxforce", maxSpringForce, response)) {
				printf("--- ERROR: set mySpring maxforce: %s\n", response);
			}
		}

		// Create the Haptic BiasForce Effect and supply it with parameters
		if (haSendCommand(dev, "create biasforce myBiasForce", response)) {
			printf("--- ERROR: Could not send command create biasforce myBiasForce\n");
		}
		printf("create biasforce myBiasForce ==> %s\n", response);

		// set but do not enable a bias force.
		if (strstr(response, "--- ERROR:")) {
			getchar();
			exit(-1);
		}
		else {
			haSendCommand(dev, "set myBiasForce force", 0.0, 0.0, 0.0, response);
		}


		// Create planes to keep the wrist within the workspace

		double HMLimits[6];
		double boundaryStiffness = 500;


		int j;

		FILE *limitFile;
		limitFile = fopen("HMlimits.txt", "r");
		for (j = 0; j<6; j++) {
			// read the target order
			//fscanf(targetOrderFile,"%d",&targetOrder[j]);
			//printf("Target %d: ",targetOrder[j]);

			// read the targets and save them to the variable target
			fscanf(limitFile, "%lf", &HMLimits[j]);
		}
		fclose(limitFile);


		printf("Create planes\n");
		if (haSendCommand(dev, "create flatplane planex", response)) {
			printf("--- ERROR: Could not send command create planex\n");
		}
		haSendCommand(dev, "set planex pos", HMLimits[0], 0, 0, response);
		haSendCommand(dev, "set planex normal [-1,0,0]", response);
		haSendCommand(dev, "set planex stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planex enable", response);


		if (haSendCommand(dev, "create flatplane planex2", response)) {
			printf("--- ERROR: Could not send command create planex2\n");
		}
		haSendCommand(dev, "set planex2 pos", HMLimits[1], 0, 0, response);
		haSendCommand(dev, "set planex2 normal [1,0,0]", response);
		haSendCommand(dev, "set planex2 stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planex2 enable", response);


		if (haSendCommand(dev, "create flatplane planey", response)) {
			printf("--- ERROR: Could not send command create planey\n");
		}
		haSendCommand(dev, "set planey pos", 0, HMLimits[2], 0, response);
		haSendCommand(dev, "set planey normal [0,-1,0]", response);
		haSendCommand(dev, "set planey stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planey enable", response);

		if (haSendCommand(dev, "create flatplane planey2", response)) {
			printf("--- ERROR: Could not send command create planey2\n");
		}
		haSendCommand(dev, "set planey2 pos", 0, HMLimits[3], 0, response);
		haSendCommand(dev, "set planey2 normal [0,1,0]", response);
		haSendCommand(dev, "set planey2 stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planey2 enable", response);



		if (haSendCommand(dev, "create flatplane planez", response)) {
			printf("--- ERROR: Could not send command create planex\n");
		}
		haSendCommand(dev, "set planez pos", 0, 0, HMLimits[4], response);
		haSendCommand(dev, "set planez normal [0,0,-1]", response);
		haSendCommand(dev, "set planez stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planez enable", response);

		if (haSendCommand(dev, "create flatplane planez2", response)) {
			printf("--- ERROR: Could not send command create planex\n");
		}
		haSendCommand(dev, "set planez2 pos", 0, 0, HMLimits[5], response);
		haSendCommand(dev, "set planez2 normal [0,0,1]", response);
		haSendCommand(dev, "set planez2 stiffness", boundaryStiffness, response);
		haSendCommand(dev, "set planez2 enable", response);

		printf("Planes created at [%f %f %f %f %f %f]\n",HMLimits[0],HMLimits[1], HMLimits[2], HMLimits[3], HMLimits[4], HMLimits[5]);
	}
}

//---------------------------------------------------------------------
//                          Init_Opto
//
// This function initializes the Optotrak
//---------------------------------------------------------------------
void Init_Opto (void)
{
	unsigned int uFlags, uElements, uFrameNumber;

	printf("...TransputerLoadSystem\n" );
    if( TransputerLoadSystem( "system" ) != OPTO_NO_ERROR_CODE )
    {
        error=1;
        goto ERROR_EXIT;
    }    
    sleep( 1 );

	printf("...TransputerInitializeSystem\n" );
    if( TransputerInitializeSystem( OPTO_LOG_ERRORS_FLAG | OPTO_LOG_MESSAGES_FLAG ) )
    {
        error=1;
        goto ERROR_EXIT;
    }
    
    printf("...OptotrakSetProcessingFlags\n" );
    if( OptotrakSetProcessingFlags( OPTO_LIB_POLL_REAL_DATA |
                                    OPTO_CONVERT_ON_HOST |
                                    OPTO_RIGID_ON_HOST ) )
    {
        error=1;
        goto ERROR_EXIT;
    }
    

	printf("...OptotrakLoadCameraParameters \n" );
	if( OptotrakLoadCameraParameters(CAMFILE) ) //if filename==NULL, standard.cam is loaded
	{
		 error=1;
		 goto ERROR_EXIT;
	}

	printf("...OptotrakSetStroberPortTable\n" );
	if( OptotrakSetStroberPortTable(NUM_MARKERS_PORT1,NUM_MARKERS_PORT2,NUM_MARKERS_PORT3,NUM_MARKERS_PORT4)){
		error = 1;
		goto ERROR_EXIT;
	}
	printf("...OptotrakSetupCollection\n" );
    if( OptotrakSetupCollection(
            NUM_MARKERS,        /* Number of markers in the collection. */
            FREQUENCY,        /* Frequency to collect data frames at. */
            1700,        /* Marker frequency for marker maximum on-time. */
            30,                 /* Dynamic or Static Threshold value to use. */
            160,                /* Minimum gain code amplification to use. */
            0,                  /* Stream mode for the data buffers. */
            (float)0.25,        /* Marker Duty Cycle to use. */
            (float)11.05,         /* Voltage to use when turning on markers. */
            (float)1.0,         /* Number of seconds of data to collect. */
            (float)0.0,         /* Number of seconds to pre-trigger data by. */
            OPTOTRAK_NO_FIRE_MARKERS_FLAG) )
    {
        error=1;
        goto ERROR_EXIT;
    }
    sleep( 1 );

	printf("...OptotrakActivateMarkers\n" );
    if( OptotrakActivateMarkers() )
    {
        error=1;
        goto ERROR_EXIT;
    }    

	printf("\n...Rigid body definition\n" );
    // Define the thorax rigid body
    printf("...Define thorax\n" );  
    if( RigidBodyAddFromFile
				(
            0,         			/* ID associated with this rigid body. */
            1,    /* First marker in the rigid body.*/
            THORAX_FILE,         			    /* RIG file containing rigid body coordinates.*/
            OPTOTRAK_RETURN_QUATERN_FLAG ) ) /* flags */
        {
             error=1;
             goto ERROR_EXIT;
        }
    
    if( RigidBodyChangeSettings
				 (
             0,    /* ID associated with this rigid body. */
             3,              /* Minimum number of markers which must be seen
                                before performing rigid body calculations.*/
             60,             /* Cut off angle for marker inclusion in calcs.*/
             (float)0.5,      /* Maximum 3-D marker error for this rigid body. */
             (float)0.2,     /* Maximum raw sensor error for this rigid body. */
             (float)0.5,     /* Maximum 3-D RMS marker error for this rigid body. */
             (float)0.1,     /* Maximum raw sensor RMS error for this rigid body. */
              OPTOTRAK_RETURN_QUATERN_FLAG ) )
         {
             error=1;
             goto ERROR_EXIT;
         }
    
	// Define the humerus rigid body
	printf("...Define humerus\n");
	if (RigidBodyAddFromFile
	(
		1,         			/* ID associated with this rigid body. */
		FIRST_HUMERUS,    /* First marker in the rigid body.*/
		HUMERUS_FILE,         			    /* RIG file containing rigid body coordinates.*/
		OPTOTRAK_RETURN_QUATERN_FLAG)) /* flags */
	{
		error = 1;
		goto ERROR_EXIT;
	}
	if (RigidBodyChangeSettings
	(
		1,    /* ID associated with this rigid body. */
		3,              /* Minimum number of markers which must be seen
						before performing rigid body calculations.*/
		60,             /* Cut off angle for marker inclusion in calcs.*/
		(float)0.5,      /* Maximum 3-D marker error for this rigid body. */
		(float)0.2,     /* Maximum raw sensor error for this rigid body. */
		(float)0.5,     /* Maximum 3-D RMS marker error for this rigid body. */
		(float)0.1,     /* Maximum raw sensor RMS error for this rigid body. */
		OPTOTRAK_RETURN_QUATERN_FLAG))
	{
		error = 1;
		goto ERROR_EXIT;
	}
    
    // Define the forearm rigid body
    printf("...Define forearm\n" );  
    if( RigidBodyAddFromFile
				(
            2,         			/* ID associated with this rigid body. */
            FIRST_FOREARM,    /* First marker in the rigid body.*/
            FOREARM_FILE,         			    /* RIG file containing rigid body coordinates.*/
            OPTOTRAK_RETURN_QUATERN_FLAG ) ) /* flags */
        {
             error=1;
             goto ERROR_EXIT;
        }
    if( RigidBodyChangeSettings
				 (
             2,    /* ID associated with this rigid body. */
             3,              /* Minimum number of markers which must be seen
                                before performing rigid body calculations.*/
             60,             /* Cut off angle for marker inclusion in calcs.*/
             (float)0.5,      /* Maximum 3-D marker error for this rigid body. */
             (float)0.2,     /* Maximum raw sensor error for this rigid body. */
             (float)0.5,     /* Maximum 3-D RMS marker error for this rigid body. */
             (float)0.1,     /* Maximum raw sensor RMS error for this rigid body. */
              OPTOTRAK_RETURN_QUATERN_FLAG ) )
         {
             error=1;
             goto ERROR_EXIT;
         }

	
	// get the data
	printf("...getting data\n");
	if(DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, pData6d, pData3d))
	{
        error=1;
		printf("error getting latest optotrak data.");
		//freeMemory(5,nRigidBodies,nIREDPositions);
		goto ERROR_EXIT;
	} 

	/**********************************************************************
     * Report optotrak error messages
     **********************************************************************/ 
    ERROR_EXIT:
    if (error)
    {
        printf("\nAn error has occurred during execution of the Optotrak program.\n" );
        if( OptotrakGetErrorString( szNDErrorString,
                                    MAX_ERROR_STRING_LENGTH + 1 ) == 0 )
        {
           printf(szNDErrorString );
        }

        printf("\n\n...TransputerShutdownSystem\n" );
        OptotrakDeActivateMarkers();
        TransputerShutdownSystem();
    }
}

//---------------------------------------------------------------------
//                          Init_UDP
//
// This function initializes UDP communication between the host computer
// and the xpc target computer
//---------------------------------------------------------------------

int Init_UDP (void)
{
	int nRetCode = 0;

	// initialize MFC and print and error on failure
	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: MFC initialization failed\n"));
		nRetCode = 1;
	}
	
	else
	{
		int retval;

		//WSAData socket_data;
		// Set these IP addresses to match your setup, Server = xPC Target, Client = xPC Host for your purpose
		CString strServerAddress(_T( TARGETIP ));
		CString strClientAddress(_T( HOSTIP ));
		
		//if( !AfxSocketInit(&socket_data) )
		if( !AfxSocketInit() )
			fprintf(stdout,"AfxSocketInit failed\n");
		else
			fprintf(stdout,"Socket initiated\n");

		if( !UDP_Receiver.Create(20000, SOCK_DGRAM, strClientAddress) )
		{
			retval = GetLastError();
			fprintf(stdout,"UDP create return %d\n",retval);
		}
		else
			fprintf(stdout,"...UDP socket create succeeded for receiver socket \n");

		if( !UDP_Sender.Create(25000, SOCK_DGRAM, strClientAddress) )
		{
			retval = GetLastError();
			fprintf(stdout,"UDP create return %d\n",retval);
		}
		else
			fprintf(stdout,"...UDP socket create succeeded for sender socket\n");

		if( !UDP_Sender.Connect(strServerAddress, 25000) )
		{
			retval = GetLastError();
			fprintf(stdout,"UDP connect return %d\n",retval);
		}
		else
			fprintf(stdout,"...UDP connect succeeded\n");

	}
	
	return nRetCode;
}

//---------------------------------------------------------------------
//                          T I M E R   C B
//
// This Is A Timer Function Which Call The HapticMASTER To Get The
// New EndEffector Position
//---------------------------------------------------------------------
void TimerCB (int iTimer)
{

   
   // Get The Current EndEffector Position From THe HapticMASTER
   if ( haSendCommand( dev, "get modelpos", response ) ) {
      printf("--- ERROR: get modelpos: %s\n", response);
   }
   else {
      ParseFloatVec( response, CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ] ); 
   }

   // Set The Timer For This Function Again
   glutTimerFunc (10, TimerCB, 1);
}

//--------------------------------------------------------------
// updateWristPos
//
// This funciton updates the position of the wrist
//--------------------------------------------------------------
void updateWristPos(void)
{
	int i;
	int currentState = 0;
	double dataout[NUM_RIGID_BODIES][RIGID_OUTPUT_WIDTH];
	double posout[NUM_MARKERS][MARKER_OUTPUT_WIDTH];
	// Get the Optotrak data
	if (DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, pData6d, pData3d))
	{
		error = 1;
		goto ERROR_EXIT;
	}
	i = 2; //Get wrist (forearm) position only
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if (pData6d[i].flags & OPTOTRAK_UNDETERMINED_FLAG)
		{
			dataout[i][7] = 0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			dataout[i][0] = pData6d[i].transformation.quaternion.rotation.q0;
			dataout[i][1] = pData6d[i].transformation.quaternion.rotation.qx;
			dataout[i][2] = pData6d[i].transformation.quaternion.rotation.qy;
			dataout[i][3] = pData6d[i].transformation.quaternion.rotation.qz;
			dataout[i][4] = pData6d[i].transformation.quaternion.translation.x;
			dataout[i][5] = pData6d[i].transformation.quaternion.translation.y;
			dataout[i][6] = pData6d[i].transformation.quaternion.translation.z;
			dataout[i][7] = 1;
		}
	}
	wristPos[0] = dataout[2][4];
	wristPos[1] = dataout[2][5];
	wristPos[2] = dataout[2][6];
	wristPos[3] = dataout[2][7]; //flag if visible


	for (i = 0; i<NUM_MARKERS; i++)
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if (pData3d[i].x > 10000 || pData3d[i].x < -10000) // if something went wrong, set the marker values to zero
		{
			posout[i][0] = 0;
			posout[i][1] = 0;
			posout[i][2] = 0;
			posout[i][3] = 0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			posout[i][0] = pData3d[i].x;
			posout[i][1] = pData3d[i].y;
			posout[i][2] = pData3d[i].z;
			posout[i][3] = 1;
		}
	}
	// Get the current state from the HapticMaster
	if (haSendCommand(dev, "get state", response)) {
		printf("--- ERROR: get state: %s\n", response);
	}
	else {
		currentState = strcmp(response, "\"force\"");
	}
	// Get The Current EndEffector Position From THe HapticMASTER
	if (haSendCommand(dev, "get measpos", response)) {
		printf("--- ERROR: get measpos: %s\n", response);
	}
	// save the HM position data
	ParseFloatVec(response, CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ]);

	// Get The Current EndEffector force From THe HapticMASTER
	if (haSendCommand(dev, "get measforce", response)) {
		printf("--- ERROR: get measforce: %s\n", response);
	}
	// save the HM force data
	ParseFloatVec(response, CurrentForce[PosX], CurrentForce[PosY], CurrentForce[PosZ]);

	
/**********************************************************************
		* Report optotrak error message
**********************************************************************/
ERROR_EXIT:
	if (error)
	{
		//printf("\nAn error has occurred during execution of the Optotrak program.\n" );
		if (OptotrakGetErrorString(szNDErrorString,
			MAX_ERROR_STRING_LENGTH + 1) == 0)
		{
			printf(szNDErrorString);
		}

		//printf("\n\n...TransputerShutdownSystem\n" );
		OptotrakDeActivateMarkers();
		TransputerShutdownSystem();
	}
}


//---------------------------------------------------------------------
//                          sendUDP
//
// This function sends a UDP packet
//---------------------------------------------------------------------

void sendUDP (float stim, float muscle)
{
    // HapticMaster declarations
    int retval1;
	int i, j, count;
	int currentState = 0;
	double dataout[NUM_RIGID_BODIES][RIGID_OUTPUT_WIDTH];
	double posout[NUM_MARKERS][MARKER_OUTPUT_WIDTH];

	// Get the current state from the HapticMaster
	if ( haSendCommand( dev, "get state", response ) ) {
        printf("--- ERROR: get state: %s\n", response);
    }
    else {
        currentState = strcmp(response,"\"force\"");
    }
	// Get The Current EndEffector Position From THe HapticMASTER
    if ( haSendCommand( dev, "get measpos", response ) ) {
        printf("--- ERROR: get measpos: %s\n", response);
    }
	// save the HM position data
    ParseFloatVec( response, CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ] ); 

	// Get The Current EndEffector Position in cylindrical coords From THe HapticMASTER
    if ( haSendCommand( dev, "get measposjoint", response ) ) {
        printf("--- ERROR: get measposjoint: %s\n", response);
    }
	// save the HM position cylindrical coords data
    ParseFloatVec( response, CurrentPositionCyl[PosX], CurrentPositionCyl[PosY], CurrentPositionCyl[PosZ] ); 

	// Get The Current EndEffector Velocity From THe HapticMASTER
    if ( haSendCommand( dev, "get modelvel", response ) ) {
        printf("--- ERROR: get modelvel: %s\n", response);
    }
	// save the HM velocity data
    ParseFloatVec( response, CurrentVelocity[PosX], CurrentVelocity[PosY], CurrentVelocity[PosZ] ); 

	// Get The Current EndEffector Acceleration From THe HapticMASTER
    if ( haSendCommand( dev, "get modelacc", response ) ) {
        printf("--- ERROR: get modelacc: %s\n", response);
    }
	// save the HM acceleration data
    ParseFloatVec( response, CurrentAcceleration[PosX], CurrentAcceleration[PosY], CurrentAcceleration[PosZ] ); 
	
	// Get The Current EndEffector force From THe HapticMASTER
    if ( haSendCommand( dev, "get measforce", response ) ) {
        printf("--- ERROR: get measforce: %s\n", response);
    }

	// Get The Current EndEffector force From THe HapticMASTER
    if ( haSendCommand( dev, "get measforce", response ) ) {
        printf("--- ERROR: get measforce: %s\n", response);
    }
	// save the HM force data
    ParseFloatVec( response, CurrentForce[PosX], CurrentForce[PosY], CurrentForce[PosZ] ); 
	
	// Get the Optotrak data
	if(DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, pData6d, pData3d))
	{
		error = 1;
		goto ERROR_EXIT;
	} 

	for(i=0; i<NUM_RIGID_BODIES; i++)
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if(pData6d[i].flags & OPTOTRAK_UNDETERMINED_FLAG)
		{
			dataout[i][7]=0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			dataout[i][0] = pData6d[i].transformation.quaternion.rotation.q0;
			dataout[i][1] = pData6d[i].transformation.quaternion.rotation.qx;
			dataout[i][2] = pData6d[i].transformation.quaternion.rotation.qy;
			dataout[i][3] = pData6d[i].transformation.quaternion.rotation.qz;
			dataout[i][4] = pData6d[i].transformation.quaternion.translation.x;
			dataout[i][5] = pData6d[i].transformation.quaternion.translation.y;
			dataout[i][6] = pData6d[i].transformation.quaternion.translation.z;
			dataout[i][7] = 1;
		}   
	}
	wristPos[0] = dataout[1][4];
	wristPos[1] = dataout[1][5];
	wristPos[2] = dataout[1][6];
	wristPos[3] = dataout[1][7];

	if( DataGetLatest3D( &uFrameNumber,&uElements,&uFlags,pData3d ) )
	{
		error = 1;
		goto ERROR_EXIT;
	}

	for(i=0; i<NUM_MARKERS; i++)
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if(pData3d[i].x > 10000 || pData3d[i].x < -10000) // if something went wrong, set the marker values to zero
		{
			posout[i][ 0] = 0;
			posout[i][ 1] = 0;
			posout[i][ 2] = 0;
			posout[i][3]=0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			posout[i][ 0] = pData3d[i].x;
			posout[i][ 1] = pData3d[i].y;
			posout[i][ 2] = pData3d[i].z;
			posout[i][ 3] = 1;
		}   
	}
    
	
	// assign values to the stuff to be sent
	udpData[0] = 1000*(CurrentPosition[PosX]);// + HM_XOFFSET; // HM x pos in mm
	udpData[1] = 1000*(CurrentPosition[PosY]);// + HM_YOFFSET; // HM y pos in mm
	udpData[2] = 1000*(CurrentPosition[PosZ]);// + HM_ZOFFSET; // HM z pos in mm
	udpData[3] = 1000*(CurrentPositionCyl[PosX]); // HM r pos in mm
	udpData[4] = (CurrentPositionCyl[PosY]); // HM phi pos in rad
	udpData[5] = 1000*(CurrentPositionCyl[PosZ]); // HM z pos in mm
	udpData[6] = 1000*(CurrentVelocity[PosX]); // HM x vel in mm/s
	udpData[7] = 1000*(CurrentVelocity[PosY]); // HM y vel in mm/s
	udpData[8] = 1000*(CurrentVelocity[PosZ]); // HM z vel in mm/s
	udpData[9] = 1000*(CurrentAcceleration[PosX]); // HM x acc in mm/s/s
	udpData[10] = 1000*(CurrentAcceleration[PosY]); // HM y acc in mm/s/s
	udpData[11] = 1000*(CurrentAcceleration[PosZ]); // HM z acc in mm/s/s
	udpData[12] = CurrentForce[PosX]; // HM x force
	udpData[13] = CurrentForce[PosY]; // HM y force
	udpData[14] = CurrentForce[PosZ]; // HM z force

	count = 14;
	for(i=0; i<NUM_RIGID_BODIES; i++){
		for(j=0; j<RIGID_OUTPUT_WIDTH; j++){
			count = count + 1;
			udpData[count] = dataout[i][j];
		}
	}
	
    // marker positions
	for(i=0; i<NUM_MARKERS; i++){
		for(j=0; j<MARKER_OUTPUT_WIDTH; j++){
			count = count + 1;
			udpData[count] = posout[i][j];
		}
		//printf("Marker  = %d\t X = %f\t Y = %f\t Z = %f\t Flag = %f\n",i+1,posout[i][0],posout[i][1],posout[i][2],posout[i][3]);
	}
	
	// stim information
	//count = count + 1;
	//udpData[count] = stim;
	//count = count + 1;
	//udpData[count] = muscle;

	// send the UDP packets
	// The second parameter of Send is the buffer size, but its in bytes (i.e. units of 8 bits)
	// - a 3-element array of 32bit ints has a size of 12 (3 * (32bits * (1byte/8bits)) = 12bytes)
	// - a double has a size of 8 (64bits * (1byte/8bits) = 8bytes)
	retval1 = UDP_Sender.Send(&udpData, 8*packetSize);
	if( retval1 == SOCKET_ERROR )
	{
		retval1 = GetLastError();
		fprintf(stdout,"error code %d\n",retval1);
	}
	else
		//fprintf(stdout,"%d characters sent\n",retval1);

	

	/**********************************************************************
     * Report optotrak error messages
     **********************************************************************/ 
    ERROR_EXIT:
    if (error)
    {
        //printf("\nAn error has occurred during execution of the Optotrak program.\n" );
        if( OptotrakGetErrorString( szNDErrorString,
                                    MAX_ERROR_STRING_LENGTH + 1 ) == 0 )
        {
           printf(szNDErrorString );
        }

        //printf("\n\n...TransputerShutdownSystem\n" );
        OptotrakDeActivateMarkers();
        TransputerShutdownSystem();
    }

}

//---------------------------------------------------------------------
//                          receiveUDP
//
// This function receives a UDP packet of three doubles
//---------------------------------------------------------------------

void receiveUDP (void)
{
	int retval1;
	// receive the UDP packets
	// The second parameter of Send is the buffer size, but its in bytes (i.e. units of 8 bits)
	// - a 3-element array of 32bit ints has a size of 12 (3 * (32bits * (1byte/8bits)) = 12bytes)
	// - a double has a size of 8 (64bits * (1byte/8bits) = 8bytes)
	retval1 = UDP_Receiver.Receive(&udpReceiveData, 8*receivePacketSize,0);
	if( retval1 == SOCKET_ERROR )
	{
		retval1 = GetLastError();
		fprintf(stdout,"error code %d\n",retval1);
	}
}

//---------------------------------------------------------------------
//                          moveHapticMasterTo
//
// This function moves the HapticMaster to a desired position
//---------------------------------------------------------------------
void moveHapticMasterTo(double xdes, double ydes, double zdes, double newStiffness, double newDamping)
{

	// set the position of the spring
	if ( haSendCommand(dev, "set mySpring pos", xdes, ydes, zdes, response) ) {
			printf("--- ERROR: set mySpring pos: %s\n", response);
	}
	// set the damping of the spring
	if ( haSendCommand(dev, "set myDamper dampcoef", newDamping, response) ) {
			printf("--- ERROR: set myDamper dampcoef: %s\n", response);
	}
	// set the stiffness of the spring
	if ( haSendCommand(dev, "set mySpring stiffness", newStiffness, response) ) {
			printf("--- ERROR: set mySpring stiffness: %s\n", response);
	}
	// enable the spring
	if ( haSendCommand(dev, "set mySpring enable", response) ) {
			printf("--- ERROR: set mySpring enable: %s\n", response);
	}

}

//---------------------------------------------------------------------
//                          readTarget
//
// This function reads the target files 
// and saves the muscle order, target order, and target coordinates to
// variables.
//---------------------------------------------------------------------
void readTarget(void)
{
	int i, j;

	FILE *targetFile;
	targetFile = fopen("Target.txt", "r");
	//FILE *targetOrderFile;
	//targetOrderFile = fopen("targetOrder.txt","r");
	for (j = 0; j<TARGETS; j++) {
		// read the target order
		//fscanf(targetOrderFile,"%d",&targetOrder[j]);
		//printf("Target %d: ",targetOrder[j]);

		// read the targets and save them to the variable target
		fscanf(targetFile, "%lf", &target[j][0]);
		fscanf(targetFile, "%lf", &target[j][1]);
		fscanf(targetFile, "%lf", &target[j][2]);
		//printf("x = %lf,y = %lf,z = %lf\n",target[j][0],target[j][1],target[j][2]);

		// read the muscle order
		//printf("Muscles: ");
		//printf("\n");
	}
	fclose(targetFile);





}
//---------------------------------------------------------------------
//                          executeTarget
//
// This function stimulates muscles at a given target
//---------------------------------------------------------------------
void executeTarget(void)
{
	int MoveRobot = 1; // 0 if robot should not be a part of experiment

	int i;
	int index;

	double stiff;
	double handCurrent[3];
	double posDes[3];
	double targetPos[3];
	double theTime;
	double moveTime;
	
	double err[3];
	float dt = 0.001;


	// STIM TIMING VARIABLES
	double FreeFloat = 0;
	double holdTime = 0.75;
	double FreeFloatTime = 2; //1 second after stimulation starts
	double endFloatTime = 7;
	double endTime = 4.9;
	int controlFlag = 0;

	// Passive force variables
	double UDPforce[3];
	UDPforce[0] = 0;
	UDPforce[1] = 0;
	UDPforce[2] = 0;


	// Get Target Position
	readTarget();

	////////////////////////////////////////////////////////
	// MOVE HAPTIC MASTER TO THE TARGET
	////////////////////////////////////////////////////////
	moveTime = 5;

	// Find wrist position error
	updateWristPos();
	err[0] = (target[0][0] - wristPos[0])/1000;
	err[1] = (target[0][1] - wristPos[1])/1000;
	err[2] = (target[0][2] - wristPos[2])/1000;
	printf("err = [%f,%f,%f] flag: %f\n", err[0], err[1], err[2], wristPos[3]);
	// set the HapticMaster state to force
	if (haSendCommand(dev, "set state force", response)) {
		printf("--- ERROR: set state force: %s\n", response);
	}
	while ((fabs(err[0]) > 0.01 || fabs(err[1]) > 0.01 || fabs(err[2])>0.01) &&  moveTime>.5 && wristPos[3]==1)
	{
		// deinfe the target position of the HapticMaster by adding the error to the current position
		targetPos[0] = err[0] + CurrentPosition[0];
		targetPos[1] = err[1] + CurrentPosition[1];
		targetPos[2] = err[2] + CurrentPosition[2];

		// start the clock
		index = 0;
		timer.Reset();
		timer.Start();
		// move slowly to the target
		while (timer.Read() < moveTime) {
			index = ceil(timer.Read() / dt);

			posDes[0] = CurrentPosition[0] + index*dt*(targetPos[0] - CurrentPosition[0]) / moveTime;
			posDes[1] = CurrentPosition[1] + index*dt*(targetPos[1] - CurrentPosition[1]) / moveTime;
			posDes[2] = CurrentPosition[2] + index*dt*(targetPos[2] - CurrentPosition[2]) / moveTime;
			stiff = middleStiffness + index*dt*(hardStiffness - middleStiffness) / moveTime;
			moveHapticMasterTo(posDes[0], posDes[1], posDes[2], stiff, moveDamping);
			//updateWristPos();
			//printf("Target Position = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
			//printf("Wrist Position = [%f,%f,%f] flag: %f\n", wristPos[0], wristPos[1], wristPos[2], wristPos[3]);
			//printf("current goal = [%f,%f,%f]\n", posDes[0], posDes[1], posDes[2]);
			//printf("final goal = [%f,%f,%f]\n", targetPos[0], targetPos[1], targetPos[2]);
		}
		timer.Stop();

		// set the HapticMaster state to hold at targetPos
		moveHapticMasterTo(targetPos[0], targetPos[1], targetPos[2], hardStiffness, moveDamping);

		moveTime = moveTime/2;

		// Find wrist position error
		updateWristPos();
		err[0] = (target[0][0] - wristPos[0]) / 1000;
		err[1] = (target[0][1] - wristPos[1]) / 1000;
		err[2] = (target[0][2] - wristPos[2]) / 1000;
		printf("Target Position = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
		printf("Wrist Position = [%f,%f,%f] flag: %f\n", wristPos[0], wristPos[1], wristPos[2], wristPos[3]);
		printf("err = [%f,%f,%f] flag: %f\n", err[0], err[1], err[2], wristPos[3]);
	}

	if (haSendCommand(dev, "set state stop", response)) {
		printf("--- ERROR: set state stop: %s\n", response);
	}

	printf("\nMove Done\n");
	printf("Target Position = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
	printf("Wrist Position = [%f,%f,%f] flag: %f\n", wristPos[0], wristPos[1], wristPos[2], wristPos[3]);

	printf("Waiting for XPC . . . \n");
	receiveUDP();
	receiveUDP();
	receiveUDP();
	receiveUDP();
	controlFlag = udpReceiveData[0];
	printf("Control Flag = %lf \n", controlFlag);
	while (controlFlag != 1) {
		receiveUDP();
		controlFlag = udpReceiveData[0];
		//printf("Control Flag = %lf \n",controlFlag);
	}
	if (haSendCommand(dev, "set state force", response)) {
		printf("--- ERROR: set state force: %s\n", response);
	}
	printf("Starting Stimulations\n");

	
	timer.Reset();
	timer.Start();

	// moveHapticMasterTo(targetPos[0], targetPos[1], targetPos[2], hardStiffness, moveDamping); // This seemed to cause issues where targetPos would be incorrect.
	while (timer.Read() < holdTime) {
		sendUDP(0, 0); 
		receiveUDP(); 
		controlFlag = udpReceiveData[0];
		
	}
	moveHapticMasterTo(targetPos[0], targetPos[1], targetPos[2], 0.0, 20.0);
	

	while (timer.Read() < endTime) {
		 sendUDP(0, 0);
	receiveUDP(); 
	controlFlag = udpReceiveData[0]; 

	// Provide passive force
	receiveUDP();
	UDPforce[0] = udpReceiveData[1];
	UDPforce[1] = udpReceiveData[2];
	UDPforce[2] = udpReceiveData[3];
	printf("%f\n", UDPforce[0]);
	haSendCommand(dev, "set myBiasForce force", UDPforce[0], UDPforce[1], UDPforce[2], response);
	haSendCommand(dev, "set myBiasForce enable", response);
	}

	timer.Stop();

	if (haSendCommand(dev, "set state stop", response)) {
		printf("--- ERROR: set state stop: %s\n", response);
	}

	while (controlFlag>0) { receiveUDP(); controlFlag = udpReceiveData[0]; printf("Control Flag = %lf \n", controlFlag); }
	printf("Trial complete. . . \n");

	if (haSendCommand(dev, "set state stop", response)) {
		printf("--- ERROR: set state stop: %s\n", response);
	}

	

}

//---------------------------------------------------------------------
//                          streamData
//
// This function streams data to the xpc target
//---------------------------------------------------------------------
void streamData (void)
{
	int status_hit = 0;
	
	while(status_hit == 0)
	{
		printf("I'm streaming\n");
		status_hit = _kbhit();
		
		sendUDP(0,0);
		// print the HapticMaster position
		printf("HM x: %f y: %f z: %f\n",CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ] );
	} 
}



//---------------------------------------------------------------------
//                         printData
//
// This function gets the latest HM and opto data and prints it to the screen
//---------------------------------------------------------------------
void printData (void)
{
	// HapticMaster declarations
	int i;
	int currentState = 0;
	double dataout[NUM_RIGID_BODIES][RIGID_OUTPUT_WIDTH];
	double posout[NUM_MARKERS][MARKER_OUTPUT_WIDTH];

	// Get the current state from the HapticMaster
	if ( haSendCommand( dev, "get state", response ) ) {
        printf("--- ERROR: get state: %s\n", response);
    }
    else {
        currentState = strcmp(response,"\"force\"");
    }
	// Get The Current EndEffector Position From THe HapticMASTER
    if ( haSendCommand( dev, "get measpos", response ) ) {
        printf("--- ERROR: get measpos: %s\n", response);
    }
	// save the HM position data
    ParseFloatVec( response, CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ] ); 

	// print the HapticMaster position
	printf("HM x: %f y: %f z: %f\n",CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ] );

	// Get The Current EndEffector Velocity From THe HapticMASTER
    if ( haSendCommand( dev, "get modelvel", response ) ) {
        printf("--- ERROR: get modelvel: %s\n", response);
    }
	// save the HM velocity data
    ParseFloatVec( response, CurrentVelocity[PosX], CurrentVelocity[PosY], CurrentVelocity[PosZ] ); 

	// Get The Current EndEffector Acceleration From THe HapticMASTER
    if ( haSendCommand( dev, "get modelacc", response ) ) {
        printf("--- ERROR: get modelacc: %s\n", response);
    }
	// save the HM acceleration data
    ParseFloatVec( response, CurrentAcceleration[PosX], CurrentAcceleration[PosY], CurrentAcceleration[PosZ] ); 
	
	// Get The Current EndEffector force From THe HapticMASTER
    if ( haSendCommand( dev, "get measforce", response ) ) {
        printf("--- ERROR: get measforce: %s\n", response);
    }

	// save the HM force data
    ParseFloatVec( response, CurrentForce[PosX], CurrentForce[PosY], CurrentForce[PosZ] ); 
	
	// print the HapticMaster force
	printf("HM fx: %f fy: %f fz: %f\n",CurrentForce[PosX], CurrentForce[PosY], CurrentForce[PosZ] );

	// Get the Optotrak data
	if(DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, pData6d, pData3d))
	{
		error = 1;
		goto ERROR_EXIT;
	} 

	for(i=0; i<NUM_RIGID_BODIES; i++)
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if(pData6d[i].flags & OPTOTRAK_UNDETERMINED_FLAG)
		{
			dataout[i][12]=0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			dataout[i][ 0] = pData6d[i].transformation.rotation.matrix[0][0];
			dataout[i][ 1] = pData6d[i].transformation.rotation.matrix[0][1];
			dataout[i][ 2] = pData6d[i].transformation.rotation.matrix[0][2];
			dataout[i][ 3] = pData6d[i].transformation.rotation.matrix[1][0];
			dataout[i][ 4] = pData6d[i].transformation.rotation.matrix[1][1];
			dataout[i][ 5] = pData6d[i].transformation.rotation.matrix[1][2];
			dataout[i][ 6] = pData6d[i].transformation.rotation.matrix[2][0];
			dataout[i][ 7] = pData6d[i].transformation.rotation.matrix[2][1];
			dataout[i][ 8] = pData6d[i].transformation.rotation.matrix[2][2];
			dataout[i][ 9] = pData6d[i].transformation.rotation.translation.x;
			dataout[i][10] = pData6d[i].transformation.rotation.translation.y;
			dataout[i][11] = pData6d[i].transformation.rotation.translation.z;
			dataout[i][12] = 1;
		}   
	}

	if( DataGetLatest3D( &uFrameNumber,&uElements,&uFlags,pData3d ) )
	{
		error = 1;
		goto ERROR_EXIT;
	}

	for(i=0; i<NUM_MARKERS; i++)
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if(pData3d[i].x > 10000 || pData3d[i].x < -10000) // if something went wrong, set the marker values to zero
		{
			posout[i][ 0] = 0;
			posout[i][ 1] = 0;
			posout[i][ 2] = 0;
			posout[i][3]=0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			posout[i][ 0] = pData3d[i].x;
			posout[i][ 1] = pData3d[i].y;
			posout[i][ 2] = pData3d[i].z;
			posout[i][ 3] = 1;
		}   
		// print marker flags
		printf("Marker %d flag: %f\n",i+1,posout[i][3]);
	}

	// print Optotrak rigid body data
	printf("Thorax Position x: %f y: %f z: %f\n",dataout[0][9], dataout[0][10], dataout[0][11] );
	printf("Thorax Flag: %f\n",dataout[0][12]);
	printf("Humerus Position x: %f y: %f z: %f\n",dataout[1][9], dataout[1][10], dataout[1][11] );
	printf("Humerus Flag: %f\n",dataout[1][12]);
	printf("Forearm Position x: %f y: %f z: %f\n",dataout[2][9], dataout[2][10], dataout[2][11] );
	printf("Forearm Flag: %f\n",dataout[2][12]);
	printf("HM Position Opto x: %f y: %f z: %f\n",dataout[3][9], dataout[3][10], dataout[3][11] );
	printf("HM Flag: %f\n",dataout[3][12]);
	// print marker flags


	/**********************************************************************
     * Report optotrak error messages
     **********************************************************************/ 
    ERROR_EXIT:
    if (error)
    {
        printf("\nAn error has occurred during execution of the Optotrak program.\n" );
        if( OptotrakGetErrorString( szNDErrorString,
                                    MAX_ERROR_STRING_LENGTH + 1 ) == 0 )
        {
           printf(szNDErrorString );
        }

        printf("\n\n...TransputerShutdownSystem\n" );
        OptotrakDeActivateMarkers();
        TransputerShutdownSystem();
    }
}


//---------------------------------------------------------------------
//                          stopAndRecord
//
// This function sets the HapticMaster to stop and saves the current 
// position to file
//---------------------------------------------------------------------
void stopAndRecord(void)
{
	FILE *targetFile;
	FILE *targetFile2;

	// set the HapticMaster state to stop
	if (haSendCommand(dev, "set state stop", response)) {
		printf("--- ERROR: set state stop: %s\n", response);
	}

	// Get The Current EndEffector Position From THe HapticMASTER
	if (haSendCommand(dev, "get measpos", response)) {
		printf("--- ERROR: get measpos: %s\n", response);
	}
	// save the HM position data
	ParseFloatVec(response, CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ]);

	updateWristPos();

	// open a file to save targets to
	if (wristPos[3] == 1) {
		targetFile = fopen("Hull.txt", "a");
		fprintf(targetFile, "%f\t%f\t%f\n", wristPos[PosX], wristPos[PosY], wristPos[PosZ]);
		fclose(targetFile);
		writeCount = writeCount + 1;
		printf("Point %d recorded: x = %f y = %f z = %f\n", writeCount, wristPos[PosX], wristPos[PosY], wristPos[PosZ]);

		// record Haptic Master targets
		targetFile2 = fopen("targets.txt", "a");
		fprintf(targetFile2, "%f\t%f\t%f\n", CurrentPosition[PosX], CurrentPosition[PosY], CurrentPosition[PosZ]);
		fclose(targetFile2);
	
	}
	else {
		printf("Wrist not visible\n");
	}
}


//---------------------------------------------------------------------
//                          getPassiveConfig
//
// This function moves to a position and records the rigid body orientations
//---------------------------------------------------------------------
void getPassiveConfig(void)
{

	int i;
	int index;
	int k;
	int j;

	double stiff;
	double handCurrent[3];
	double posDes[3];
	double targetPos[3];
	double theTime;
	double moveTime;
	double err[3];
	float dt = 0.001;
	double dataout[NUM_RIGID_BODIES][RIGID_OUTPUT_WIDTH];

	FILE *targetFile;


	// STIM TIMING VARIABLES
	double FreeFloat = 0;
	double FreeFloatTime = 2; //1 second after stimulation starts
	double endFloatTime = 7;
	double endTime = 12;
	int controlFlag = 0;




	////////////////////////////////////////////////////////
	// MOVE HAPTIC MASTER TO THE TARGET
	////////////////////////////////////////////////////////
	moveTime = 5;

	// Find wrist position error
	updateWristPos();
	err[0] = (target[0][0] - wristPos[0]) / 1000;
	err[1] = (target[0][1] - wristPos[1]) / 1000;
	err[2] = (target[0][2] - wristPos[2]) / 1000;
	printf("err = [%f,%f,%f] flag: %f\n", err[0], err[1], err[2], wristPos[3]);
	// set the HapticMaster state to force
	if (haSendCommand(dev, "set state force", response)) {
		printf("--- ERROR: set state force: %s\n", response);
	}
	while ((fabs(err[0]) > 0.01 || fabs(err[1]) > 0.01 || fabs(err[2])>0.01) && moveTime>.5 && wristPos[3] == 1)
	{
		// deinfe the target position of the HapticMaster by adding the error to the current position
		targetPos[0] = err[0] + CurrentPosition[0];
		targetPos[1] = err[1] + CurrentPosition[1];
		targetPos[2] = err[2] + CurrentPosition[2];

		// start the clock
		index = 0;
		timer.Reset();
		timer.Start();
		// move slowly to the target
		while (timer.Read() < moveTime) {
			index = ceil(timer.Read() / dt);

			posDes[0] = CurrentPosition[0] + index*dt*(targetPos[0] - CurrentPosition[0]) / moveTime;
			posDes[1] = CurrentPosition[1] + index*dt*(targetPos[1] - CurrentPosition[1]) / moveTime;
			posDes[2] = CurrentPosition[2] + index*dt*(targetPos[2] - CurrentPosition[2]) / moveTime;
			stiff = middleStiffness + index*dt*(hardStiffness - middleStiffness) / moveTime;
			moveHapticMasterTo(posDes[0], posDes[1], posDes[2], stiff, moveDamping);

		}
		timer.Stop();

		// set the HapticMaster state to hold at targetPos
		moveHapticMasterTo(targetPos[0], targetPos[1], targetPos[2], hardStiffness, moveDamping);

		moveTime = moveTime / 2;

		// Find wrist position error
		updateWristPos();
		err[0] = (target[0][0] - wristPos[0]) / 1000;
		err[1] = (target[0][1] - wristPos[1]) / 1000;
		err[2] = (target[0][2] - wristPos[2]) / 1000;
		printf("Target Position = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
		printf("Wrist Position = [%f,%f,%f] flag: %f\n", wristPos[0], wristPos[1], wristPos[2], wristPos[3]);
		printf("err = [%f,%f,%f] flag: %f\n", err[0], err[1], err[2], wristPos[3]);
	}

	if (haSendCommand(dev, "set state stop", response)) {
		printf("--- ERROR: set state stop: %s\n", response);
	}

	printf("\nMove Done\n");
	printf("Target Position = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
	printf("Wrist Position = [%f,%f,%f] flag: %f\n", wristPos[0], wristPos[1], wristPos[2], wristPos[3]);

	// Get passive forces
	printf("Getting Passive Force...\n");
	timer.Reset();
	timer.Start();
	while (timer.Read() < 1) {
		updateWristPos();
		passiveForce[0] = CurrentForce[0];
		passiveForce[1] = CurrentForce[1];
		passiveForce[2] = CurrentForce[2];
	}
	printf("Passive Force = [%f %f %f]\n", passiveForce[0], passiveForce[1], passiveForce[2]);
	printf("Done\n");


	targetFile = fopen("passive.txt", "a");
	// Get the Optotrak data
	if (DataGetLatestTransforms2(&uFrameNumber, &uElements, &uFlags, pData6d, pData3d))
	{
		error = 1;
		goto ERROR_EXIT;
	}
	for(i=0;i<3;i++) //Get wrist (forearm) position only
	{
		//printf("Rigid body: %d Flag: %f.\n",i+1,pData6d[i].flags);
		// check if the rigid body is visible and set the 'lost visibility' flag if it isn't
		if (pData6d[i].flags & OPTOTRAK_UNDETERMINED_FLAG)
		{
			dataout[i][7] = 0;
		}
		else
		{
			//printf("Getting data for rigid body %d.\n",i+1);
			dataout[i][0] = pData6d[i].transformation.quaternion.rotation.q0;
			dataout[i][1] = pData6d[i].transformation.quaternion.rotation.qx;
			dataout[i][2] = pData6d[i].transformation.quaternion.rotation.qy;
			dataout[i][3] = pData6d[i].transformation.quaternion.rotation.qz;
			dataout[i][4] = pData6d[i].transformation.quaternion.translation.x;
			dataout[i][5] = pData6d[i].transformation.quaternion.translation.y;
			dataout[i][6] = pData6d[i].transformation.quaternion.translation.z;
			dataout[i][7] = 1;
		}
	}

	targetFile = fopen("passive.txt", "a");
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 8; j++) {
			fprintf(targetFile, "%f\t", dataout[i][j]);
		}
	}
	fprintf(targetFile, "%f\t%f\t%f\n", passiveForce[0], passiveForce[1], passiveForce[2]);
	fclose(targetFile);


	/**********************************************************************
	* Report optotrak error message
	**********************************************************************/
ERROR_EXIT:
	if (error)
	{
		//printf("\nAn error has occurred during execution of the Optotrak program.\n" );
		if (OptotrakGetErrorString(szNDErrorString,
			MAX_ERROR_STRING_LENGTH + 1) == 0)
		{
			printf(szNDErrorString);
		}

		//printf("\n\n...TransputerShutdownSystem\n" );
		OptotrakDeActivateMarkers();
		TransputerShutdownSystem();
	}

}




//---------------------------------------------------------------------
//                            K E Y B O A R D
//
// This Function Is Called By OpenGl WhenEver A Key Was Hit
//---------------------------------------------------------------------
void Keyboard(unsigned char ucKey, int iX, int iY)
{
	int hulltargs;

	FILE *targetFile;
	FILE *targetFile2;
	int i;
	int junk;

	double targets[35][3]; // update this to match number of hull targets
	

   switch (ucKey) 
   {


      case 27:  // esc turns everything off
	     // shutdown HapticMaster
         if ( haSendCommand(dev, "remove all", response) ) {
            printf("---ERROR: remove all: %s\n", response);
         }
         if ( haSendCommand(dev, "set state stop", response) ) {
            printf("---ERROR: set state stop: %s\n", response);
         }
		
         if ( haDeviceClose(dev) ) {
            printf("---ERROR: closing device\n");
         }
		 
		 // shutdown Optotrak
		 printf("\n\n...TransputerShutdownSystem\n" );
         OptotrakDeActivateMarkers();
         TransputerShutdownSystem();

		 // close the UDP sockets
		 UDP_Sender.Close();
		 UDP_Receiver.Close();

         exit(0);
         break;

	  case 'e':  // e executes muscle stimulations at the current target and then increments to the next target
		  printf("target: %d\n",targetNumber);
		  if (targetNumber <= TARGETS)

			executeTarget();
		  printf("target complete\n");

		  break;

	  case 't':  // t executes muscle stimulations at a user-defined target and then increments to the next target
		  printf("Input target to execute: ");
		  scanf("%d",&targetNumber);
		  printf("target: %d\n",targetNumber);
		  if (targetNumber <= TARGETS)
		  {
			executeTarget();
			printf("target %d complete\n",targetNumber);
			targetNumber = targetNumber + 1;
		  }
		  else
			printf("You are done.  Press ESC to exit.");
		  break;
		  break;

	  case 'd':  // d prints out on block of data
		  printData();
		  break;

	  case 's':  // s streams data to xpc target until another key is pressed
		  streamData();
		  break;

	  case 'g': // get Target Hull
				 // open a file to save targets to
		  targetFile = fopen("Hull.txt", "w+");
		  targetFile2 = fopen("targets.txt", "w+");
		  hulltargs = 27;
		  writeCount = 0;
		  while (writeCount < hulltargs)
		  {
			  // set the HapticMaster state to force
			  if (haSendCommand(dev, "set state force", response)) {
				  printf("--- ERROR: set state force: %s\n", response);
			  }
			  printf("Move to target, type 1, and press enter. \n");
			  scanf("%d", &junk);
			  stopAndRecord();
			  sleep(1);
		  }
		  printf("Done. Hit Escape\n");
		  break;

	  case 'f': // move to each position and record config
		  hulltargs = 35; // Update to be the number of targets in the final HullTargets.txt
						// update size of targets as well

		  targetFile = fopen("HullTargets.txt", "r");
		  for (i = 0; i < hulltargs; i++) {
			  fscanf(targetFile, "%lf", &targets[i][0]);
			  fscanf(targetFile, "%lf", &targets[i][1]);
			  fscanf(targetFile, "%lf", &targets[i][2]);
		  }

		  targetFile2 = fopen("passive.txt", "w+");
		  for (i = 0; i < hulltargs; i++) {
			  target[0][0] = targets[i][0];
			  target[0][1] = targets[i][1];
			  target[0][2] = targets[i][2];
			  printf("TARGET = [%f,%f,%f]\n", target[0][0], target[0][1], target[0][2]);
			  getPassiveConfig();
		  }
		  break;

	  case 'o': // o shuts everything off, but doesn't kill the program
		  // shutdown HapticMaster
         if ( haSendCommand(dev, "remove all", response) ) {
            printf("---ERROR: remove all: %s\n", response);
         }
         if ( haSendCommand(dev, "set state stop", response) ) {
            printf("---ERROR: set state stop: %s\n", response);
         }
		
         if ( haDeviceClose(dev) ) {
            printf("---ERROR: closing device\n");
         }
		 
		 // shutdown Optotrak
		 printf("\n\n...TransputerShutdownSystem\n" );
         OptotrakDeActivateMarkers();
         TransputerShutdownSystem();

		 // close the UDP sockets
		 UDP_Sender.Close();
		 UDP_Receiver.Close();

		 break;
   }
}


//---------------------------------------------------------------------
//                              M A I N
//
// 02-Hello-Grpahic-World Main Function
//---------------------------------------------------------------------
int main(int argc, char** argv)
{
	  // Initialize the HapticMaster
	  printf("HapticMaster Initializing...");
	  Init_HM();
	  printf("done\n");

	  // Initialize the Optotrak
	  printf("\nOptotrak Initializing...");
	  Init_Opto();
      printf("done\n");

	  // UDP Setup Initialization Calls
	  printf("\nOpening UDP communications...");
	  Init_UDP();
	  printf("done\n");


      // OpenGL Initialization Calls
      glutInit(&argc, argv);
      glutInitDisplayMode (GLUT_DOUBLE| GLUT_RGB | GLUT_DEPTH);
      glutInitWindowSize (800, 600);

      // Create The OpenGlWindow
	  glutCreateWindow ("HapticAPI Programming Manual : Example02: Hello Graphic World");

      InitOpenGl();

      // More OpenGL Initialization Calls
      glutReshapeFunc (Reshape);
      glutDisplayFunc(Display);
      glutKeyboardFunc (Keyboard);
      glutTimerFunc (10, TimerCB, 1);
      glutMainLoop();

   return 0; 


}
