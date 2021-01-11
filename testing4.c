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

struct ray
{
    vec3 p;
    vec3 d;
};

/* Unsightly global variables | By: Petros Faloutsos*/
double det4x4( double m[4][4] );
double det3x3( double a1, double a2, double a3, double b1, double b2, double b3, double c1, double c2, double c3 );
double det2x2( double a, double b, double c, double d);

/* Functions declarations, Descriptions above actual functions */
void save_image(int Width, int Height, char *fname, unsigned char *pixels);
void invert_matrix(double A[4][4], double Ainv[4][4]);
void adjoint(double in[4][4], double out[4][4]);
struct container* readFile(char* fn); 

float mag(vec3 vpc);
void primaryRay(vec3 or, vec3 px, struct ray *dest);
int intersect(struct ray* ray, struct sphere * sphere, vec3* normal);
void reflect(struct ray* ray, vec3 n, struct ray* refRay);
void inverseTransform(vec3 point, vec3 d, struct sphere sph);
int quadSolver(float a, float b, float c, struct ray* ray);
float * tracer(struct ray* ray, int b, struct container * conP);
float * phong(struct container* conP, struct sphere* sphere, struct light* light, vec3 p, vec3 N, float * rayColor);

/*
* By: Damian Steiger
* Status: In Progress
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

        unsigned char pixels[4 * (*(con->so))->RES[0] * (*(con->so))->RES[1] + 1];      //ToDo why does 3 not work but 4 does?!?!

    //// End | Pixel Array ////

    //// Start | Raytracing ////

        struct ray dest;

        for (int c = 0; c < (*(con->so))->RES[1]; c++)      //traverse across the rows (y value)
        {
            for (int r = 0; r < 3*((*(con->so))->RES[0]); r += 3)      //traverse across the columns (x value)
            {
                primaryRay((vec3){0,0,0}, (vec3){(r - (((*(con->so))->RES[0])/2)), (c - (*(con->so))->RES[1]/2), -((*(con->so))->NEAR)}, &dest);   /*
                                                                                                                                                    * Calculates the primary ray. Also, translates
                                                                                                                                                    * pixel points to center of camera view
                                                                                                                                                    */
                float * color = tracer(&dest, 1, con);     //returns pixel color data ToDo, proper illumination
                pixels[c*(*(con->so))->RES[0]*3+r] = color[0] * 255;      //setting red value
                pixels[c*(*(con->so))->RES[0]*3+r + 1] = color[1] * 255;      //setting green value
                pixels[c*(*(con->so))->RES[0]*3+r + 2] = color[2] * 255;      //setting blue value
            }
            
        }
    //// End | Raytracing ////

    //// Start | Saving Image ////

        save_image((*(con->so))->RES[0], (*(con->so))->RES[1], (*(con->so))->OUTPUT,  pixels);

    //// End | Saving Image ////

    return 0;
}

/*
* By: Damian Steiger
* Status: In Progress
* DESCRIPTION ToDo
*/
float * tracer(struct ray* ray, int b, struct container * conP)
{
    if(b > MAX_BOUNCES) return (*(conP->so))->BACK;       //if max bounces then return bakcground color

    vec3 normal;
    vec3 normalCopy;
    int status = 0;
    float finalC[] = {0,0,0};
    int ctrl;

    struct ray rayCopy;
    struct ray rayList[3];
    float prev = -10;
    int statusCopy = 0;
    for (int j = 0; j < 3; j++)
    {
        glm_vec3_copy(ray->d, rayCopy.d);
        glm_vec3_copy(ray->p, rayCopy.p);
        statusCopy = intersect(&rayCopy, (conP->spa)[j], &normalCopy);
        if (rayCopy.p[2] > prev & statusCopy == 1)
        {
            *ray = rayCopy;
            status = statusCopy;
            glm_vec3_copy(normal, normalCopy);
            ctrl = j;
        }
        
    }

    if (status == 0) return (*(conP->so))->BACK;

    for (int k = 0; k < 3; k++)
    {
        float * local;
        phong(conP, (conP->spa)[ctrl], (conP->lpa)[k], ray->p, normal, local);
        finalC[0] += local[0];
        finalC[1] += local[0];
        finalC[2] += local[0];
    }

    struct ray r;
    reflect(ray, normal, &r);
    float * reflected = tracer(&r, b+1, conP);

    float* sum = malloc(sizeof(float) * 3);
    sum[0] = reflected[0] + finalC[0];
    sum[1] = reflected[1] + finalC[1];
    sum[2] = reflected[2] + finalC[2];

    if(sum[0] > 1) sum[0] = 1;
    if(sum[1] > 1) sum[1] = 1;
    if(sum[2] > 1) sum[2] = 1;

    return sum;
    //ToDo, not accoutning for object depth, just returning first object
}

/*
* By: Damian Steiger
* Status: In Progress
* DESCRIPTION ToDo
*/
float * phong(struct container* conP, struct sphere* sphere, struct light* light, vec3 p, vec3 N, float * rayColor)
{
    vec3 cen = {(*sphere).pos[0], (*sphere).pos[1], (*sphere).pos[2]};       //convert center to vector

    vec3 L;
    glm_vec3_sub(cen, p, L);

    vec3 V = {0,0,-1};

    vec3 R;
    glm_vec3_proj(L, N, R);

    glm_vec3_normalize(V);
    glm_vec3_normalize(L);
    glm_vec3_normalize(N);
    glm_vec3_normalize(R);

    float NdotL = glm_vec3_dot(N, L);
    float RdotV = glm_vec3_dot(R, V);

    float Red = ((*sphere).k[1] * (*light).intensity[0] * NdotL * (*sphere).color[0] + (*sphere).k[2] * (*light).intensity[0] * pow(RdotV, (*sphere).n)) + ((*sphere).k[0] * (*(conP->so))->AMBIENT[0] * (*sphere).color[0]) + ((*sphere).k[2] * rayColor[0]);
    float Green = ((*sphere).k[1] * (*light).intensity[1] * NdotL * (*sphere).color[1] + (*sphere).k[2] * (*light).intensity[1] * pow(RdotV, (*sphere).n)) + ((*sphere).k[0] * (*(conP->so))->AMBIENT[1] * (*sphere).color[1]) + ((*sphere).k[2] * rayColor[1]);
    float Blue = ((*sphere).k[1] * (*light).intensity[2] * NdotL * (*sphere).color[2] + (*sphere).k[2] * (*light).intensity[2] * pow(RdotV, (*sphere).n)) + ((*sphere).k[0] * (*(conP->so))->AMBIENT[2] * (*sphere).color[2]) + ((*sphere).k[2] * rayColor[2]);

    float *finalC = malloc(sizeof(float) * 3);
    finalC[0] = Red;
    finalC[1] = Green;
    finalC[2] = Blue;

    return finalC;
}

/*
* By: Damian Steiger
* Status: Done!
* Given a point, direction, container, and normal pointer, compute if ray
* hits sphere (ToDo, only works for sphere 0), and returns normal
* in the normal pointer. When finsihed d = intersection point (because i said so)
*/
int intersect(struct ray* ray, struct sphere * sphere, vec3* normal)
{
    //inverseTransform(ray->p, ray->d, *sphere);        //inverse transform ray (if the spheres have a scaling factor)ToDo and inverse transpose normal
    
    glm_vec3_normalize(ray->d);
    float a = (glm_vec3_dot(ray->d, ray->d));
    vec3 cen = {(*sphere).pos[0], (*sphere).pos[1], (*sphere).pos[2]};       //convert center to vector
    vec3 oc;
    glm_vec3_sub(ray->p,cen,oc);
    float b = 2 * (glm_vec3_dot(ray->d, oc));
    float c = glm_vec3_dot(oc,oc) - 1;

    int status = quadSolver(a, b, c, ray);  

    glm_vec3_sub(ray->p, cen, *normal);
    
    if(status == 0) return 0;
    if(status == 1) return 1;
    if(status == 2) return 1;

    return 0;
}

/*
* By: Damian Steiger
* Status: Done!
* DESCRIPTION ToDo
*/
int quadSolver(float a, float b, float c, struct ray* ray)
{
    float determinant = ((b*b) - (4.0*a*c));

    float r1;
    float r2;

    if (determinant == 0)       //one solution
    {

        r1 = -b / (2.0 * a);
        vec3 mult;
        glm_vec3_scale(ray->d, r1, mult);
        glm_vec3_add(ray->p, mult , ray->p);
        glm_vec3_scale(ray->p, -1, ray->p);
        return 1;

    } else if (determinant > 0)     //two solutions
    {

        r1 = (-b + sqrt(determinant)) / (2.0 * a);
        r2 = (-b - sqrt(determinant)) / (2.0 * a);

        if (r1 <= r2)
        {
            vec3 mult;
            glm_vec3_scale(ray->d, r1, mult);
            glm_vec3_add(ray->p, mult , ray->p);
            //glm_vec3_scale(ray->p, -1, ray->p); ToDo, sometimes negatives and positives flipped
            return 2;
        }
        if (r1 > r2)
        {
            vec3 mult;
            glm_vec3_scale(ray->d, r2, mult);
            glm_vec3_add(ray->p, mult , ray->p);
            //glm_vec3_scale(ray->p, -1, ray->p);
            return 2;
        }
        
    } 
    return 0;
}

/*
* By: Damian Steiger
* Status: Done!
* Applies inverse transform the point and direction
* of the ray to account for scaled spheres
*/
void inverseTransform(vec3 point, vec3 d, struct sphere sph)
{
    mat4 inTrans = {sph.scale[0], 0, 0, 0,
                    0, sph.scale[1], 0, 0,
                    0, 0, sph.scale[2], 0,
                    0, 0, 0, 1};

    glm_mat4_inv(inTrans, inTrans);

    mat4 dB = {d[0],0,0,0,
                0,d[1],0,0,
                0,0,d[2],0,
                0,0,0,1};
    mat4 pointB = {point[0],0,0,0,
                0,point[1],0,0,
                0,0,point[2],0,
                0,0,0,1};

    glm_mat4_mul(inTrans, pointB, pointB);
    glm_mat4_mul(inTrans, dB, dB);

    point[0] = pointB[0][0];
    point[1] = pointB[1][1];
    point[2] = pointB[2][2];
    
    d[0] = dB[0][0];
    d[1] = dB[1][1];
    d[2] = dB[2][2];
}

/*
* By: Damian Steiger
* Status: Done!
* Calculates the Magnitude of a 1 x 3 vector
*/
float mag(vec3 vpc)
{
    return (sqrt(pow(vpc[0], 2) + pow(vpc[1], 2)+ pow(vpc[2], 2)));
}

/*
* By: Damian Steiger
* Status: Done!
* Taking a ray and a normal as input, this function
* reflects the ray about the normal, striong the
* subsequent reflection in "r"
*/
void reflect(struct ray* ray, vec3 n, struct ray* refRay)
{
    glm_vec3_scale(n, 2*(glm_vec3_dot(ray->d, n)), refRay->d);

    glm_vec3_sub(refRay->d, ray->d, refRay->d);
}

/*
* By: Damian Steiger
* Status: Done!
* Given a pixel (stored as vector for easy math), and the origin,
* return the unit vector from the origin to the pixel. Return is actually
* just stored in "dest" pointer.
*/
void primaryRay(vec3 or, vec3 px, struct ray *dest)
{
    vec3 d;
    glm_vec3_sub(px, or, d);
    glm_vec3_normalize(d);
    
    glm_vec3_copy(dest->d, d);
    glm_vec3_copy(dest->p, or);
}

/*
* By: Damian Steiger
* Status: Done!
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
*   - Intersection Function
*   - shifting pixels to allign with camera may be a problem
*   - Local Illumination Model
*   - Troubleshooting
*/

/*
links 
https://eclass.yorku.ca/eclass/pluginfile.php/1662011/mod_resource/content/2/Assignment3.pdf
https://eclass.yorku.ca/eclass/pluginfile.php/1619413/mod_resource/content/2/cse3431-lecture13-raytracing.pdf
https://cglm.readthedocs.io/en/latest/vec3.html#c.glm_vec3_copy
https://www.lighthouse3d.com/tutorials/maths/ray-sphere-intersection/
http://www.ccs.neu.edu/home/fell/CS4300/Lectures/Ray-TracingFormulas.pdf
https://eclass.yorku.ca/eclass/pluginfile.php/1701460/mod_resource/content/1/ppm.cpp
*/
