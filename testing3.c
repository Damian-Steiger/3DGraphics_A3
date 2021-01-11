

printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", rayB[0][0],rayB[0][1],rayB[0][2],rayB[0][3],rayB[1][0],rayB[1][1],rayB[1][2],rayB[1][3],rayB[2][0],rayB[2][1],rayB[2][2],rayB[2][3],rayB[3][0],rayB[3][1],rayB[3][2],rayB[3][3]);

printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", inTrans[0][0],inTrans[0][1],inTrans[0][2],inTrans[0][3],inTrans[1][0],inTrans[1][1],inTrans[1][2],inTrans[1][3],inTrans[2][0],inTrans[2][1],inTrans[2][2],inTrans[2][3],inTrans[3][0],inTrans[3][1],inTrans[3][2],inTrans[3][3]);

printf("%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n", inTrans[0][0],inTrans[0][1],inTrans[0][2],inTrans[0][3],inTrans[1][0],inTrans[1][1],inTrans[1][2],inTrans[1][3],inTrans[2][0],inTrans[2][1],inTrans[2][2],inTrans[2][3],inTrans[3][0],inTrans[3][1],inTrans[3][2],inTrans[3][3]);


printf("%f , %f , %f\n", ray[0], ray[1], ray[2]);
    printf("%f , %f , %f\n", point[0], point[1], point[2]);


for (int c = 0; c < (*(con->so))->RES[1]; c++)      //traverse across the rows (y value)
{
    for (int r = 0; r < ((*(con->so))->RES[0]); r++)      //traverse across the columns (x value)
    {
        /*primaryRay((vec3){0,0,0}, (vec3){(r - (((*(con->so))->RES[0])/2)), (c - (*(con->so))->RES[1]/2), -((*(con->so))->NEAR)}, &dest);   /*
                                                                                                                                            * Calculates the primary ray. Also, translates
                                                                                                                                            * pixel points to center of camera view
                                                                                                                                            */
        //float * color =tracer((vec3){0,0,0}, dest, 1, con);     //returns pixel color data 
        pixels[c*(*(con->so))->RES[0]*3+r] = r+c/3;      //setting pixel to color
    }
    
}


sum[0] = (local[0] + reflected[0] / 2.0);
    sum[1] = (local[1] + reflected[1]) / 2.0;
    sum[2] = (local[2] + reflected[2]) / 2.0;
    return sum;



float discriminant = (b * b) - (4.0 * a * c);

    float root1;
    float root2;

    // condition for real and different roots
    if (discriminant > 0.0) {
        root1 = (-b + sqrt(discriminant)) / (2.0 * a);
        root2 = (-b - sqrt(discriminant)) / (2.0 * a);
        if (root1 > 0.0 & root1 < root2)
        {
            vec3 mult;
            glm_vec3_scale_as(ray->d, root1, mult);
            glm_vec3_add(ray->p, mult , ray->p);
            return 2;
        }
        if (root2 > 0.0 & root2 < root1)
        {
            vec3 mult;
            glm_vec3_scale_as(ray->d, root2, mult);
            glm_vec3_add(ray->p, mult , ray->p);
            return 2;
        }
        return 0;
    }

    // condition for real and equal roots
    else if (discriminant == 0.0) {
        root1 = root2 = -b / (2.0 * a);
        vec3 mult;
        glm_vec3_scale_as(ray->d, root1, mult);
        glm_vec3_add(ray->p, mult , ray->p);
        return 1;
    }

    // if roots are not real
    else {
        printf("02\n");
        return 0;
    }

    printf("03\n");
    return 0;


     //testing
    struct sphere sphereT;
    sphereT.pos[0] = 0;
    sphereT.pos[1] = 0;
    sphereT.pos[2] = -3;
    sphereT.scale[0] = 1;
    sphereT.scale[1] = 1;
    sphereT.scale[2] = 1;
    vec3 normal;
    struct ray ray;
    ray.d[0] = 0;
    ray.d[1] = 0;
    ray.d[2] = -1;
    ray.p[0] = 0;
    ray.p[1] = 0;
    ray.p[2] = 0;
    int returns = intersect(&ray, &sphereT, &normal);
    //printf("return = %d | point = %f,%f,%f | normal = %f,%f,%f\n", returns,ray.p[0],ray.p[1],ray.p[2], normal[0], normal[1], normal[2]);
    quadSolver(2,5,3, &ray);
    printf("point = %f,%f,%f\n",ray.p[0], ray.p[1], ray.p[2]);