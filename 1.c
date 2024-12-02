#include <stdio.h>
#include <math.h>
#include <stdlib.h>

typedef struct {
    double x, y;
} cord;

typedef struct {
    int x, y;
} memb;

typedef struct {
    double x, y;
} direct;

double determinant(double **matrix, int n) {
    if (n == 1) {
        return matrix[0][0];
    }

    double det = 0;
    int sign = 1;

    double **minor = (double **)malloc((n - 1) * sizeof(double *));
    for (int i = 0; i < n - 1; i++) {
        minor[i] = (double *)malloc((n - 1) * sizeof(double));
    }

    for (int i = 0; i < n; i++) {
        for (int j = 1; j < n; j++) {
            for (int k = 0; k < n; k++) {
                if (k != i) {
                    minor[j - 1][k < i ? k : k - 1] = matrix[j][k];
                }
            }
        }
        det += sign * matrix[0][i] * determinant(minor, n - 1);
        sign *= -1;
    }

    for (int i = 0; i < n - 1; i++) {
        free(minor[i]);
    }
    free(minor);

    return det;
}

direct direction(memb m, cord *jj, int a) {
    double distance = sqrt(pow((jj[m.x].x - jj[m.y].x), 2) + pow((jj[m.x].y - jj[m.y].y), 2));
    direct sss = {0, 0};
    if (m.x == a || m.y == a) {
        sss.x = (jj[m.x].x - jj[m.y].x) / distance;
        sss.y = (jj[m.x].y - jj[m.y].y) / distance;
    }
    if (m.x == a) {
        sss.x *= -1;
        sss.y *= -1;
    }
    return sss;
}

int main() {
    int j, m;
    printf("Enter number of joints: ");
    scanf("%d", &j);
    printf("Enter number of members: ");
    scanf("%d", &m);

    if (m != 2 * j - 3) {
        printf("This program might not be able to solve it (not a simple truss).\n");
        return 0;
    }

    cord *jj = (cord *)malloc(j * sizeof(cord));
    memb *mm = (memb *)malloc(m * sizeof(memb));
    cord *extforce = (cord *)calloc(j, sizeof(cord));
    double *extfrce = (double *)calloc(2 * j, sizeof(double));
    double **force = (double **)calloc(2 * j, sizeof(double *));
    double **force0 = (double **)calloc(2 * j, sizeof(double *));

    for (int i = 0; i < 2 * j; i++) {
        force[i] = (double *)calloc(2 * j, sizeof(double));
        force0[i] = (double *)calloc(2 * j, sizeof(double));
    }

    for (int i = 0; i < j; i++) {
        printf("Enter x-coordinate of joint %d: ", i + 1);
        scanf("%lf", &jj[i].x);
        printf("Enter y-coordinate of joint %d: ", i + 1);
        scanf("%lf", &jj[i].y);
    }

    for (int i = 0; i < m; i++) {
        printf("Enter serial number of first point for member %d: ", i + 1);
        scanf("%d", &mm[i].x);
        printf("Enter second point for member %d: ", i + 1);
        scanf("%d", &mm[i].y);
        if (mm[i].x > mm[i].y) {
            int temp = mm[i].x;
            mm[i].x = mm[i].y;
            mm[i].y = temp;
        }
    }
    for (int i = 0; i < m; i++) {
        mm[i].x-=1;mm[i].y-=1;
    }

    direct roll;
    printf("Enter unit vector's x-coordinate for roller restriction: ");
    scanf("%lf", &roll.x);
    printf("Enter unit vector's y-coordinate for roller restriction: ");
    scanf("%lf", &roll.y);

    int f;
    printf("Enter number of joints you want to apply force on: ");
    scanf("%d", &f);

    for (int i = 2; i < f + 2; i++) {
        printf("Enter x-coordinate of force on joint %d: ", i + 1);
        scanf("%lf", &extforce[i].x);
        printf("Enter y-coordinate of force on joint %d: ", i + 1);
        scanf("%lf", &extforce[i].y);
    }

    for (int i = 0; i < j; i++) {
        extfrce[2 * i] = extforce[i].x;
        extfrce[2 * i + 1] = extforce[i].y;
    }

    force[0][2 * j - 3] = 1;
    force[1][2 * j - 2] = 1;
    force[2][2 * j - 1] = roll.x;
    force[3][2 * j - 1] = roll.y;

    for (int i = 0; i < j; i++) {
        for (int b = 0; b < m; b++) {
            direct uv = direction(mm[b], jj, i);
            force[2 * i][b] = uv.x;
            force[2 * i + 1][b] = uv.y;
        }
    }

    double d[2 * j], oo;
    oo = determinant(force, 2 * j);

    for (int i = 0; i < 2 * j; i++) {
        for (int k = 0; k < 2 * j; k++) {
            for (int a = 0; a < 2 * j; a++) {
                force0[k][a] = (a == i) ? extfrce[k] : force[k][a];
            }
        }
        d[i] = determinant(force0, 2 * j);
    }

    cord pin = {d[2 * j - 3] / oo, d[2 * j - 2] / oo};
    double rollfrce = fabs(d[2 * j - 1] / oo);
    printf("Force in pin joint is %.2lfx + %.2lfy N\n", -pin.x, -pin.y);
    printf("External force at roller is %.2lf N\n", rollfrce);

    double forcememb[m];
    for (int i = 0; i < m; i++) {
        forcememb[i] = d[i] / oo;
        if(forcememb[i]>0)
        {printf("Force in member %d is %.2lf N and is tension \n", i + 1, forcememb[i]);}
        else{
            printf("Force in member %d is %.2lf N and is compression \n", i + 1, fabs(forcememb[i]));
        }
    }

    for (int i = 0; i < 2 * j; i++) {
        free(force[i]);
        free(force0[i]);
    }
    free(force);
    free(force0);
    free(jj);
    free(mm);
    free(extforce);
    free(extfrce);

    return 0;
}
