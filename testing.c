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
void primaryRay(vec3 or, vec3 px, vec3 *dest);
int intersect(vec3 point, vec3 ray, struct container * conP, vec3* normal);
void reflect(vec3 ray, vec3 n, vec3* r);
void inverseTransform(vec3 point, vec3 d, struct sphere sph);
float* phongIllumination();
float* tracer(vec3 point, vec3 d, int b, struct container * conP);
int intersectLight(vec3 point, vec3 d, struct container * conP, float* light);

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

        vec3 dest;

        for (int c = 0; c < (*(con->so))->RES[1]; c++)      //traverse across the rows (y value)
        {
            for (int r = 0; r < 3*((*(con->so))->RES[0]); r += 3)      //traverse across the columns (x value)
            {
                primaryRay((vec3){0,0,0}, (vec3){(r - (((*(con->so))->RES[0])/2)), (c - (*(con->so))->RES[1]/2), -((*(con->so))->NEAR)}, &dest);   /*
                                                                                                                                                    * Calculates the primary ray. Also, translates
                                                                                                                                                    * pixel points to center of camera view
                                                                                                                                                    */
                float * color = tracer((vec3){0,0,0}, dest, 1, con);     //returns pixel color data ToDo, proper illumination
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
* Status: In Progess
* DESCRIPTION ToDo
*/
float * phongIllumination(vec3 point, struct container * conP)
{
    float* key;

    for (int k = 0; k < (sizeof(*(conP->lpa)) / sizeof(struct light)); k++)
    {
        vec3 center = {(*(conP->lpa))[k].pos[0], (*(conP->lpa))[k].pos[1], (*(conP->lpa))[k].pos[2]};

        vec3 vpc;
        glm_vec3_sub(center, point, vpc);       //vector from point to center

        vec3 useless;       //intersect needs this parameter, we wont use it here

        int status = intersect(point, vpc, conP, &useless);      //see if this point can see a light

        float *sum = malloc(sizeof(float) * 3);
        sum[0] = 0;
        sum[1] = 0;
        sum[2] = 0;

        if(status == 0){
            sum[0] += (*(conP->lpa))[0].intensity[0];
            sum[1] += (*(conP->lpa))[0].intensity[1];
            sum[2] += (*(conP->lpa))[0].intensity[2];
        }
        key = sum;
    }

    //if multiple lights add to intensity above 1 then just set it to 1
    if(key[0] > 1) key[0] = 1;
    if(key[0] > 1) key[0] = 1;
    if(key[0] > 1) key[0] = 1;

    return key;
}

/*
* By: Damian Steiger
* Status: In Progress
* DESCRIPTION ToDo
*/
float* tracer(vec3 point, vec3 d, int b, struct container * conP)
{ 
    if(b > MAX_BOUNCES) return (*(conP->so))->BACK;       //if max bounces then return bakcground color

    vec3 normal;
    int status;
    int statusL;
    float * light;

    vec3 ray;
    glm_vec3_copy(d, ray);

    status = intersect(point, d, conP, &normal);   

    statusL = intersectLight(point, d, conP, light);   
    if(statusL == 1) return light;       //if the ray hit a light source

    if(status == 0) return  (*(conP->so))->BACK;       //no intersection

    vec3 r;
    reflect(ray, normal, &r);

    float* local = phongIllumination(d, conP);      //phong illumination
    float* reflected = tracer(ray, r, b + 1, conP);

    float *sum = malloc(sizeof(float) * 3); //generalize across objects ToDo local illumination model
    sum[0] = (((*(conP->so))->AMBIENT[0] * (*(conP->spa))[0].color[0] * (*(conP->spa))[0].k[0]) + local[0] + (((*(conP->spa))[0].k[0]) * reflected[0]));
    sum[1] = (((*(conP->so))->AMBIENT[1] * (*(conP->spa))[0].color[0] * (*(conP->spa))[0].k[0]) + local[1] + (((*(conP->spa))[0].k[0]) * reflected[1]));
    sum[2] = (((*(conP->so))->AMBIENT[2] * (*(conP->spa))[0].color[0] * (*(conP->spa))[0].k[0]) + local[2] + (((*(conP->spa))[0].k[0]) * reflected[2]));
    return sum;
}

//ToDo
int intersectLight(vec3 point, vec3 d, struct container * conP, float* light)
{
    vec3 useless;       //intersect needs this parameter, we wont use it here
    if (intersect(point, d, conP, &useless) == 0)
    {
        for (int k = 0; k < (sizeof(*(conP->lpa)) / sizeof(struct light)); k++)
        {
            vec3 center = {(*(conP->lpa))[k].pos[0], (*(conP->lpa))[k].pos[1], (*(conP->lpa))[k].pos[2]};

            vec3 vpc;
            glm_vec3_sub(center, point, vpc);       //vector from point to center

            glm_vec3_scale_as(d, 1, d);      //makesure vectors are normalized
            glm_vec3_scale_as(vpc, 1, vpc);      //makesure vectors are normalized

            if(d[0] == vpc[0] & d[1] == vpc[1] & d[2] == vpc[2])        //if direction vector points at light
            {
                light[0] = (*(conP->lpa))[k].intensity[0];
                light[2] = (*(conP->lpa))[k].intensity[1];
                light[1] = (*(conP->lpa))[k].intensity[2];
                return 1;
            }
        }
    }
    return 0;
}

/*
* By: Damian Steiger
* Status: Broken
* Given a point, direction, container, and normal pointer, compute if ray
* hits sphere (ToDo, only works for sphere 0), and returns normal
* in the normal pointer. When finsihed d = intersection point (because i said so)
*/
int intersect(vec3 point, vec3 d, struct container * conP, vec3* normal)
{
    int key;        //z buffer algorithm for front sphere ToDo

    for (int k = 0; k < (sizeof(*(conP->spa)) / sizeof(struct sphere)); k++)
    {
        inverseTransform(point, d, (*(conP->spa))[k]);        //inverse transform ray (if the spheres have a scaling factor)

        vec3 center = {(*(conP->spa))[k].pos[0], (*(conP->spa))[k].pos[1], (*(conP->spa))[k].pos[2]};       //convert center to vector | ToDo dont just do one sphere
        vec3 vpc;
        glm_vec3_sub(center, point, vpc);       //vector from center to point on line

        if (glm_vec3_dot(vpc, d) < 0)
        {
            if (glm_vec3_dot(vpc, d) > 1) key = 0;      //no intersection

            else if (mag(vpc) == 1) //interection at point
            {
                vec3 pc;
                glm_vec3_proj(center, d, pc);
                glm_vec3_normalize(pc);
                glm_vec3_copy(pc, *normal);
                key = 1;       
            }
            
            else
            {
                vec3 pc;
                vec3 pcc;
                glm_vec3_proj(center, d, pc);
                glm_vec3_sub(pc, center, pcc);
                float dist = sqrt((1*1) - (mag(pcc)) * (mag(pcc)));
                float di1 = dist - mag(pcc);
                glm_vec3_scale(d, di1, d);
                glm_vec3_sub(d, point, d);      //d now holds intersection location
                glm_vec3_normalize(pc);
                glm_vec3_copy(pc, *normal);
                key = 2;       //d intersects at a single location
            }
        }
        else
        {
            vec3 pc;
            glm_vec3_proj(center, d, pc);
            vec3 cpc;
            glm_vec3_sub(center, pc, cpc);

            if (mag(cpc) > 1) key = 0;      //no intersection
            else
            {
                vec3 pcc;
                glm_vec3_sub(pc, center, pcc);
                float dist = sqrt((1*1) - (mag(pcc)) * (mag(pcc)));

                float di1;

                if (mag(vpc) > 1)
                {
                    di1 = mag(pcc) - dist;
                }
                else
                {
                    di1 = mag(pcc) + dist;
                }
                glm_vec3_scale(d, di1, d);
                glm_vec3_sub(d, point, d);      //d now holds intersection location
                glm_vec3_normalize(pc);
                glm_vec3_copy(pc, *normal);
                key = 2;       //d intersects at a single location
            }
            
        }        
    }
    return key;    
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
void reflect(vec3 ray, vec3 n, vec3* r)
{
    glm_vec3_scale(n, 2*(glm_vec3_dot(ray, n)), *r);

    glm_vec3_sub(*r, ray, *r);
}

/*
* By: Damian Steiger
* Status: Done!
* Given a pixel (stored as vector for easy math), and the origin,
* return the unit vector from the origin to the pixel. Return is actually
* just stored in "dest" pointer.
*/
void primaryRay(vec3 or, vec3 px, vec3 *dest)
{
    glm_vec3_sub(px, or, *dest);
    glm_vec3_normalize(*dest);
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
