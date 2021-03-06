/*
* By: Damian Steiger (with code by Petros Faloutsos)
* STD #: 216433476
* Title: Assignment 3
*
* Ray tracing engine which takes scene and object information from
* file and outputs ppm file of raytraced scene.
*/

#include <cglm/cglm.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

/* Constants */
#define SMALL_NUMBER 1.e-8
#define MAX_BOUNCES 3

/* Unsightly global variables | By: Petros Faloutsos*/
double det4x4( double m[4][4] );
double det3x3( double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3 );
double det2x2( double a, double b, double c, double d);

/* Functions declarations, Descriptions above actual functions */
void save_image(int Width, int Height, char *fname, unsigned char *pixels);
void invert_matrix(double A[4][4], double Ainv[4][4]);
void adjoint(double in[4][4], double out[4][4]);
struct container* readFile(char* fn); 

float* tracer(vec3 point, vec3 ray, int b, struct container * conP);
vec3* intersect(vec3 p, vec3 ray, vec3* normal, struct container * conP);
void primaryRay(vec3 or, vec3 px, vec3 dest);
void reflect(vec3 ray, vec3 n, vec3 *ans);
float* findLocalIllumination(vec3 q, struct container * conP);

/* 
* By: Damian Steiger
*
* Class for scene objects
*
* The near plane, left, right, top, and bottom
* The resolution of the image nColumns X nRows
* The background color
* The scene’s ambient intensity
* The output file name (you should limit this to 20 characters with no spaces)
*/
struct scene
{ 
    float NEAR;
    float LEFT;
    float RIGHT;
    float BOTTOM;
    float TOP;
    int RES[2];
    float BACK[3];      //Format: (x, y), between 0 and 1
    float AMBIENT[3];   //Format: (Ir, Ig, Ib), between 0 and 1
    char OUTPUT[20];
};

/*
* By: Damian Steiger
*
* class for sphere objects
*
* The position and scaling (non-uniform), color, Ka, Kd, Ks, Kr and the specular exponent n of a sphere
*/
struct sphere
{
    char name[20];
    float pos[3];       //Format: (x, y, z)
    float scale[3];     //Format: (x, y, z)
    float color[3];     //Format: (r, g, b)
    float k[4];         //Format: (Ka, Kd, Ks, Kr), between 0 and 1
    int n;
};

/*
* By: Damian Steiger
*
* class for light soucre objects
*
* The position and intensity of a point light source
*/
struct light
{ 
    char name[20];
    float pos[3];           //Format: (x, y, z)
    float intensity[3];     //Format: (Ir, Ig, Ib)
};

/*
* By: Damian Steiger
*
* class to hold all other objects for easy passing around
*
* so: pointer to poitner of scene object
* spa: pointer to pointer to of spehre array
* lpa: pointer to pointer to light array
*/
struct container
{
    struct scene **so;
    struct sphere **spa;
    struct light **lpa; 
};

/*
* By: Damian Steiger
*
* Load file data into objects for scene, sphere, and lights. Then,
* raytrace the scene, giving color info to each pixel. Finally, send 
* rgb pixel info to ppm file. 
*/
int main(int argc, char *argv[])
{

    //// Start | Load ////

        struct container* con = readFile(argv[1]);      /*
                                                        * get file name, send it to the file read function, which   
                                                        * reads file line by line, adding the info to objects (scene,
                                                        * sphere, and light), then passes object refrences back here
                                                        */

    //// End | Load ////                                               

    //// Start | Pixel Array ////

        unsigned char pixels[3 * (*(con->so))->RES[0] * (*(con->so))->RES[1]];

    //// End | Pixel Array ////

    //// Start | Raytracing ////

        vec3 dest;

        for (int c = 0; c < (*(con->so))->RES[0]; c++)      //traverse across the columns (x value)
        {
            for (int r = 0; r < (*(con->so))->RES[1]; r++)      //traverse across the rows (y value)
            {
                primaryRay((vec3){0,0,0}, (vec3){(c - (((*(con->so))->RES[0])/2)), (r - (*(con->so))->RES[1]), -((*(con->so))->RES[0])}, dest);     /*
                                                                                                                                                    * Calculates the primary ray. Also, translates
                                                                                                                                                    * pixel points to center of camera view
                                                                                                                                                    */
                //tracer((vec3){0,0,0}, dest, 1, con);     //returns pixel color data
            }
            
        }
    
    //// End | Raytracing ////

    //// Start | Saving Image ////

        //save_image((*(con->so))->RES[0], (*(con->so))->RES[1], (*(con->so))->OUTPUT,  pixels);

    //// End | Saving Image ////

    /// Testing ////

    return 0;
}

/*
* By: Damian Steiger
*
* DESCRIPTION ToDo
*/
void primaryRay(vec3 or, vec3 px, vec3 dest)
{
    glm_vec3_sub(px, or, dest);
    glm_vec3_normalize(dest);
}

/*
* By: Damian Steiger
*
* DESCRIPTION ToDo
*/
float* tracer(vec3 point, vec3 ray, int b, struct container * conP)
{
    if(b > MAX_BOUNCES) return (*(conP->so))->BACK;       //if max bounces then return bakcground color

    vec3 normal;       //make sure normal returns as unit ToDo
    vec3 q = intersect(point, ray, &normal, conP);

    for(int i = 0; i < (sizeof(*(conP->lpa)) / sizeof(struct light)); i++)      //loop through all the lights, seeing if the ray hit any of them
    {
        if(*q == (conP->lpa)[i]->pos) return (conP->lpa)[i]->intensity;      //if you hit a light then returns that light's intensity
    }
        

    if(q == (vec3){100,100,100}) return (*(conP->so))->BACK;        //arbitrary point behind camera to signify nothing was hit

    vec3 ans;
    reflect(ray, *normal, &ans);

    float * local = findLocalIllumination(q, conP);
    float * reflected = tracer(q, ans, b + 1, conP);

    float color[3];

    for(int i = 0; i < (sizeof(*(conP->lpa)) / sizeof(struct light)); i++)      //loop through all the lights
    {
        for (int i = 0; i < 3; i++)
        {
            color[i] = local[i] + (reflected[i] * (*(conP->lpa))->intensity[i]); //ToDo, implement proper intensity model
        }
    }
    
    return color;
}

/*
* By: Damian Steiger
*
* DESCRIPTION ToDo
*/
float* findLocalIllumination(vec3 q, struct container * conP)
{
    float ans[] = {1,1,1};
    return ans;        //ToDo, proper local illumination model
}

/*
* By: Damian Steiger
*
* DESCRIPTION ToDo
*/
void reflect(vec3 ray, vec3 n, vec3* ans)
{
    glm_vec3_scale(n, 2*(glm_vec3_dot(ray, n)), *ans);

    glm_vec3_sub(ray, *ans, *ans);
}

/*
* By: Damian Steiger
*
* DESCRIPTION ToDo
*/
vec3* intersect(vec3 p, vec3 ray, vec3* normal, struct container * conP) //ToDo, current only works for one spehere, generalize
{

    
    
}

//--------------------------------Do not cross for now... (ToDo)--------------------------------------------------

/*
* By: Damian Steiger
*
* Loads the text file, reading info into structure objects
*/
struct container* readFile(char *fn)
{
    FILE *f = fopen(fn, "r");

    if(!f)  //if file open fails
    {
        printf("Failed to open file! Exiting.\n");
        exit(1);     
    }

    char* fc = (char*)malloc((1000)* sizeof(char));     //ToDo, size "1000" is arbitrary
    fc[0] = '\0';

    struct scene *so = (struct scene*)malloc(sizeof(struct scene));     //create scene object
    struct sphere *spa[15];      //sphere array
    struct light *lpa[10];      //sphere array
    for(int i = 0 ; i < 10; i++)        //allocating memory for light array
    {
        lpa[i] = (struct light*)malloc(sizeof(struct light));
    }
    int j = 0;
    for(int i = 0 ; i < 15; i++)        //allocating memory for sphere array
    {
        spa[i] = (struct sphere*)malloc(sizeof(struct sphere));
    }
    int i = 0;

    while(!feof(f))
	{
		char line[100];
		if(fgets(line,100,f)!=NULL)                     //ToDo, size "100" is arbitrary
		{
			strncat(fc,line,strlen(line));

            char field[20];
            sscanf(line, "%s", field);      //identify the field on this line

            ////Start | Populate Scene Object////

                if(strcmp(field, "NEAR") == 0)      //if the field is "NEAR"
                {
                    sscanf(line,"%s %f", field, &(so->NEAR));
                }

                if(strcmp(field, "LEFT") == 0)      //if the field is "LEFT"
                {
                    sscanf(line,"%s %f", field, &(so->LEFT));
                }

                if(strcmp(field, "RIGHT") == 0)      //if the field is "RIGHT"
                {
                    sscanf(line,"%s %f", field, &(so->RIGHT));
                }

                if(strcmp(field, "BOTTOM") == 0)      //if the field is "BOTTOM"
                {
                    sscanf(line,"%s %f", field, &(so->BOTTOM));
                }

                if(strcmp(field, "TOP") == 0)      //if the field is "TOP"
                {
                    sscanf(line,"%s %f", field, &(so->TOP));
                }

                if(strcmp(field, "RES") == 0)      //if the field is "RES"
                {
                    sscanf(line,"%s %d %d", field, &(so->RES[0]), &(so->RES[1]));
                }

                if(strcmp(field, "BACK") == 0)      //if the field is "BACK"
                {
                    sscanf(line,"%s %f %f %f", field, &(so->BACK[0]), &(so->BACK[1]), &(so->BACK[2]));
                    //printf("%f |%f |%f \n", so->BACK[0], so->BACK[1], so->BACK[2]);
                }

                if(strcmp(field, "AMBIENT") == 0)      //if the field is "AMBIENT"
                {
                    sscanf(line,"%s %f %f %f", field, &(so->AMBIENT[0]), &(so->AMBIENT[1]), &(so->AMBIENT[2]));
                    //printf("%f |%f |%f \n", so->AMBIENT[0], so->AMBIENT[1], so->AMBIENT[2]);
                }

                if(strcmp(field, "OUTPUT") == 0)      //if the field is "OUTPUT"
                {
                    sscanf(line,"%s %s", field, (so->OUTPUT));
                    //printf("%s\n", so->OUTPUT);
                }

            ////End | Populate Scene Object////

            ////Start | Populate Sphere Object(s)////

                if(strcmp(field, "SPHERE") == 0)      //if the field is "SPHERE"
                {   
                    sscanf(line,"%s %s %f %f %f %f %f %f %f %f %f %f %f %f %f %d", field, (spa[i]->name), &(spa[i]->pos[0]), &(spa[i]->pos[1]),
                                                                                        &(spa[i]->pos[2]), &(spa[i]->scale[0]), &(spa[i]->scale[1]),
                                                                                        &(spa[i]->scale[2]), &(spa[i]->color[0]), &(spa[i]->color[1]),
                                                                                        &(spa[i]->color[2]), &(spa[i]->k[0]), &(spa[i]->k[1]),
                                                                                        &(spa[i]->k[2]), &(spa[i]->k[3]), &spa[i]->n);                                                                 
                    i++;
                }

            ////End | Populate Sphere Object(s)////

            ////Start | Populate Light Object(s)////

                if(strcmp(field, "LIGHT") == 0)      //if the field is "LIGHT"
                {   

                    sscanf(line,"%s %s %f %f %f %f %f %f", field, (lpa[j]->name), &(lpa[j]->pos[0]),
                                                            &(lpa[j]->pos[1]), &(lpa[j]->pos[2]),
                                                            &(lpa[j]->intensity[0]), &(lpa[j]->intensity[1]),
                                                            &(lpa[j]->intensity[2]));
                    j++;
                }

            ////End | Populate Light Object(s)////
		}
	}

    fclose(f);

     struct container *con = (struct container*)malloc(sizeof(struct container));     //create container object
     con->so = &so;
     con->lpa = lpa;
     con->spa = spa;

    return con;
}

/* 
* By: Petros Faloutsos
*
* Converts pixel information to ppm file
* 
* Width: width value of the image to be written
* Height: height value of the image to be written
* fname: char array containing name of file to be written
* pixels: char array of pixel information
*/
void save_image(int Width, int Height, char *fname, unsigned char *pixels)
{
    FILE *fp;
    const int maxVal = 255;

    printf("Saving image %s: %d x %d\n", fname, Width, Height);
    fp = fopen(fname, "wb");
    if (!fp)
    {
        printf("Unable to open file '%s'\n", fname);
        return;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", Width, Height);
    fprintf(fp, "%d\n", maxVal);

    for (int j = 0; j < Height; j++)
    {
        fwrite(&pixels[j * Width * 3], 3, Width, fp);
    }

    fclose(fp);
}

/* 
* By: Petros Faloutsos
*
* Given two matrices; inverts the first, stores result in second
* 
* A: matrix to be inverted
* Ainv: matrix to store result
*/
void invert_matrix(double A[4][4], double Ainv[4][4])
{
    int i, j;
    double det;

    adjoint(A, Ainv);

    det = det4x4(A);

    if (fabs(det) < SMALL_NUMBER)
    {
        fprintf(stderr, "invert_matrix: matrix is singular!");
        return;
    }

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            Ainv[i][j] = Ainv[i][j] / det;
}

/* 
* By: Petros Faloutsos
*
* Aids "invert_matrix" 
* 
* in: matrix to be adjoined
* out: adjoined matrix
*/
void adjoint(double in[4][4], double out[4][4])
{
    double a1, a2, a3, a4, b1, b2, b3, b4;
    double c1, c2, c3, c4, d1, d2, d3, d4;

    a1 = in[0][0];
    b1 = in[0][1];
    c1 = in[0][2];
    d1 = in[0][3];

    a2 = in[1][0];
    b2 = in[1][1];
    c2 = in[1][2];
    d2 = in[1][3];

    a3 = in[2][0];
    b3 = in[2][1];
    c3 = in[2][2];
    d3 = in[2][3];

    a4 = in[3][0];
    b4 = in[3][1];
    c4 = in[3][2];
    d4 = in[3][3];

    out[0][0] = det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4);
    out[1][0] = -det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4);
    out[2][0] = det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4);
    out[3][0] = -det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4);

    out[0][1] = -det3x3(b1, b3, b4, c1, c3, c4, d1, d3, d4);
    out[1][1] = det3x3(a1, a3, a4, c1, c3, c4, d1, d3, d4);
    out[2][1] = -det3x3(a1, a3, a4, b1, b3, b4, d1, d3, d4);
    out[3][1] = det3x3(a1, a3, a4, b1, b3, b4, c1, c3, c4);

    out[0][2] = det3x3(b1, b2, b4, c1, c2, c4, d1, d2, d4);
    out[1][2] = -det3x3(a1, a2, a4, c1, c2, c4, d1, d2, d4);
    out[2][2] = det3x3(a1, a2, a4, b1, b2, b4, d1, d2, d4);
    out[3][2] = -det3x3(a1, a2, a4, b1, b2, b4, c1, c2, c4);

    out[0][3] = -det3x3(b1, b2, b3, c1, c2, c3, d1, d2, d3);
    out[1][3] = det3x3(a1, a2, a3, c1, c2, c3, d1, d2, d3);
    out[2][3] = -det3x3(a1, a2, a3, b1, b2, b3, d1, d2, d3);
    out[3][3] = det3x3(a1, a2, a3, b1, b2, b3, c1, c2, c3);
}

double det4x4(double m[4][4])
{
    double ans;
    double a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4;

    a1 = m[0][0];
    b1 = m[0][1];
    c1 = m[0][2];
    d1 = m[0][3];

    a2 = m[1][0];
    b2 = m[1][1];
    c2 = m[1][2];
    d2 = m[1][3];

    a3 = m[2][0];
    b3 = m[2][1];
    c3 = m[2][2];
    d3 = m[2][3];

    a4 = m[3][0];
    b4 = m[3][1];
    c4 = m[3][2];
    d4 = m[3][3];

    ans = a1 * det3x3(b2, b3, b4, c2, c3, c4, d2, d3, d4) - b1 * det3x3(a2, a3, a4, c2, c3, c4, d2, d3, d4) + c1 * det3x3(a2, a3, a4, b2, b3, b4, d2, d3, d4) - d1 * det3x3(a2, a3, a4, b2, b3, b4, c2, c3, c4);
    return ans;
}

//double det3x3( a1, a2, a3, b1, b2, b3, c1, c2, c3 )
//   double a1, a2, a3, b1, b2, b3, c1, c2, c3;
double det3x3(double a1, double a2, double a3, double b1, double b2, double b3, double c1,
              double c2, double c3)
{
    double ans;

    ans = a1 * det2x2(b2, b3, c2, c3) - b1 * det2x2(a2, a3, c2, c3) + c1 * det2x2(a2, a3, b2, b3);
    return ans;
}

//double det2x2( a, b, c, d)
//   double a, b, c, d;
double det2x2(double a, double b, double c, double d)
{
    double ans;
    ans = a * d - b * c;
    return ans;
}


/*
* ToDo (ctrl + f "ToDo"), (ctrl + f "WORKING", to find where you left off)
*
*   - Compile with just "make"
*   - Compute Primary ray
*   - Trace rays recursively
*   - Intersection Function
*   - shifting pixels to allign with camera may be a problem
*   - potential optimization probelm with passing all the scene info the the raytracer everytime
*   - return light color instead of intensity? fix lighting combination in general
*/