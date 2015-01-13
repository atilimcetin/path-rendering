#define _USE_MATH_DEFINES
#include <math.h>

#include "pr.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include <stdint.h>
#include "kvec.h"
#include "khash.h"
#include "svgpathparser.h"

#pragma comment(lib, "glew32.lib")

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

/* begin of mesh merger */

struct vector2f
{
    float x, y;
};

struct vector4f
{
    float x, y, z, w;
};

#define kh_vector2f_hash_equal(a, b) ((a.x) == (b.x) && (a.y) == (b.y))
#define kh_vector4f_hash_equal(a, b) ((a.x) == (b.x) && (a.y) == (b.y) && (a.z) == (b.z) && (a.w) == (b.w))
#define kh_vector2f_hash_func(a) ((khint_t)((a.x) * 1e6f) ^ (khint_t)((a.y) * 1e6f))
#define kh_vector4f_hash_func(a) ((khint_t)((a.x) * 1e6f) ^ (khint_t)((a.y) * 1e6f) ^ (khint_t)((a.z) * 1e6f) ^ (khint_t)((a.w) * 1e6f))

KHASH_INIT(vector2f, struct vector2f, unsigned short, 1, kh_vector2f_hash_func, kh_vector2f_hash_equal)
KHASH_INIT(vector4f, struct vector4f, unsigned short, 1, kh_vector4f_hash_func, kh_vector4f_hash_equal)

/* TODO: better hash functions */

struct merger4f
{
    khash_t(vector4f) *h;
    kvec_t(struct vector4f) vertices;
    kvec_t(unsigned short) indices;
};

void init_merger4f(struct merger4f *m)
{
    m->h = kh_init(vector4f);
    kv_init(m->vertices);
    kv_init(m->indices);
}

void free_merger4f(struct merger4f *m)
{
    kh_destroy(vector4f, m->h);
    kv_free(m->vertices);
    kv_free(m->indices);
}

static void merge4f(struct merger4f *m, const struct vector4f *vertices, unsigned short *indices, int count)
{
    int i;

    for (i = 0; i < count; ++i)
    {
        struct vector4f v;
        khiter_t k;

        v = vertices[indices[i]];

        k = kh_get(vector4f, m->h, v);

        if (k == kh_end(m->h))
        {
            int tmp;

            kv_push_back(m->indices, kv_size(m->vertices));

            k = kh_put(vector4f, m->h, v, &tmp);
            kh_value(m->h, k) = kv_size(m->vertices);

            kv_push_back(m->vertices, v);
        }
        else
        {
            kv_push_back(m->indices, kh_value(m->h, k));
        }
    }
}

/* end of mesh merger*/


#define BUFFER_OFFSET(i) ((char *)NULL + (i))

/* begin of path name management */

struct segment
{
    unsigned int start;
    int length;
    struct segment *next;
};

static struct segment *segments = NULL;

static void init_segments(void)
{
    segments = (struct segment*)malloc(sizeof(struct segment));
    segments->next = NULL;
}

static unsigned int gen_paths(int range)
{
    unsigned int start = 1;

    struct segment *prev = segments;
    struct segment *curr = segments->next;

    while (curr)
    {
        int length = curr->start - start;
        if (length >= range)
        {
            struct segment *s = (struct segment*)malloc(sizeof(struct segment));

            s->start = start;
            s->length = range;

            s->next = curr;
            prev->next = s;

            return start;
        }
        
        start = curr->start + curr->length;

        prev = curr;
        curr = curr->next;
    }

    {
        struct segment *s = (struct segment*)malloc(sizeof(struct segment));

        s->start = start;
        s->length = range;

        prev->next = s;
        s->next = NULL;

        return start;
    }
}

static void delete_path_helper(unsigned int min, unsigned int max);

static void delete_paths(unsigned int path, int range)
{
    struct segment *prev = segments;
    struct segment *curr = segments->next;

    unsigned int min1 = path;
    unsigned int max1 = path + range - 1;

    while (curr)
    {
        unsigned int min2 = curr->start;
        unsigned int max2 = curr->start + curr->length - 1;

        if (min1 > min2 && max1 < max2)
        {
            struct segment *s = (struct segment*)malloc(sizeof(struct segment));

            s->start = max1 + 1;
            s->length = max2 - max1;

            curr->length = min1 - min2;

            s->next = curr->next;
            curr->next = s;

            delete_path_helper(min1, max1);
        }
        else if (min1 <= min2 && max1 >= max2)
        {
            prev->next = curr->next;
            free(curr);
            curr = prev;

            delete_path_helper(min2, max2);
        }
        else if (min1 > min2 && min1 <= max2)
        {
            curr->length = min1 - min2;

            delete_path_helper(min1, max2);
        }
        else if (max1 >= min2 && max1 < max2)
        {
            curr->start =  max1 + 1;
            curr->length =  max2 - max1;

            delete_path_helper(min2, max1);
        }

        prev = curr;
        curr = curr->next;
    }
}

static void set_path(unsigned int path)
{
    struct segment *prev = segments;
    struct segment *curr = segments->next;

    while (curr)
    {
        unsigned int min = curr->start;
        unsigned int max = curr->start + curr->length - 1;

        if (min <= path && path <= max)
            return;

        if (path < min)
        {
            struct segment *s = (struct segment*)malloc(sizeof(struct segment));

            s->start = path;
            s->length = 1;

            s->next = curr;
            prev->next = s;

            return;
        }

        prev = curr;
        curr = curr->next;
    }

    {
        struct segment *s = (struct segment*)malloc(sizeof(struct segment));

        s->start = path;
        s->length = 1;

        prev->next = s;
        s->next = NULL;
    }
}

static void cleanup_segments(void)
{
    struct segment *curr = segments;

    while (curr)
    {
        struct segment *temp = curr;
        delete_path_helper(curr->start, curr->start + curr->length - 1);
        curr = curr->next;
        free(temp);
    }

    segments = NULL;
}

/* end of path name management */


static const char *vs_code0 =
"uniform mat4 matrix;                                                   \n"
"                                                                       \n"
#ifdef OPENGL_ES
"uniform mat4 mvp;                                                      \n"
#else
"#define mvp gl_ModelViewProjectionMatrix                               \n"
#endif
"                                                                       \n"
"attribute vec2 data0;                                                  \n"
"                                                                       \n"
"void main(void)                                                        \n"
"{                                                                      \n"
"  gl_Position = mvp * (matrix * vec4(data0, 0, 1));                    \n"
"}                                                                      \n";

static const char *fs_code0 =
"void main(void)                                               \n"
"{                                                             \n"
"  gl_FragColor = vec4(1, 1, 1, 1);                            \n"
"}                                                             \n";

static const char *vs_code1 =
"uniform mat4 matrix;                                                   \n"
"                                                                       \n"
#ifdef OPENGL_ES
"uniform mat4 mvp;                                                      \n"
#else
"#define mvp gl_ModelViewProjectionMatrix                               \n"
#endif
"                                                                       \n"
"attribute vec4 data0;                                                  \n"
"                                                                       \n"
"varying vec2 uv;                                                       \n"
"                                                                       \n"
"void main(void)                                                        \n"
"{                                                                      \n"
"  gl_Position = mvp * (matrix * vec4(data0.xy, 0, 1));   \n"
"  uv = data0.zw;                                                          \n"
"}                                                                      \n";

static const char *fs_code1 =
"varying vec2 uv;                           \n"
"                                           \n"
"void main(void)                            \n"
"{                                          \n"
"  float u = uv.x;                          \n"
"  float v = uv.y;                          \n"
"  if (u * u > v)                           \n"
"    discard;                               \n"
"  gl_FragColor = vec4(1, 1, 1, 1);         \n"
"}                                          \n";

static const char *vs_code2 =
"uniform mat4 matrix;                                                   \n"
"                                                                       \n"
#ifdef OPENGL_ES
"uniform mat4 mvp;                                                      \n"
#else
"#define mvp gl_ModelViewProjectionMatrix                               \n"
#endif
"                                                                       \n"
"attribute vec4 data0;                                                  \n"
"attribute vec4 data1;                                                  \n"
"attribute vec4 data2;                                                  \n"
"                                                                       \n"
"varying vec2 pos;                                                      \n"
"varying float p, q;                                                    \n"
"varying vec2 a, b, c;                                                  \n"
"varying float offset, strokeWidth;                                     \n"
"                                                                       \n"
"void main()                                                            \n"
"{                                                                      \n"
"  gl_Position = mvp * (matrix * vec4(data0.xy, 0, 1));   \n"
"  pos = data0.xy;                                                  \n"
"  p = data0.z;                                                         \n"
"  q = data0.w;                                                         \n"
"  a = data1.xy;                                                        \n"
"  b = data1.zw;                                                        \n"
"  c = data2.xy;                                                        \n"
"  offset = data2.z;                                                    \n"
"  strokeWidth = data2.w;                                               \n"
"}                                                                      \n";

static const char *fs_code2 =
"varying vec2 pos;                                                            \n"
"varying float p, q;                                                        \n"
"varying vec2 a, b, c;                                                        \n"
"varying float offset, strokeWidth;                                            \n"
"                                                                            \n"
"#define M_PI 3.1415926535897932384626433832795                                \n"
"                                                                            \n"
"vec2 evaluateQuadratic(float t)                                            \n"
"{                                                                            \n"
"   return a * t * t + b * t + c;                                            \n"
"}                                                                            \n"
"                                                                            \n"
"bool check(float t)                                                        \n"
"{                                                                            \n"
"   if (0.0 <= t && t <= 1.0)                                                \n"
"   {                                                                        \n"
"      vec2 q = evaluateQuadratic(t) - pos;                                    \n"
"      if (dot(q, q) <= strokeWidth)                                        \n"
"         return false;                                                        \n"
"   }                                                                        \n"
"                                                                            \n"
"   return true;                                                            \n"
"}                                                                            \n"
"                                                                            \n"
"float cbrt(float x)                                                        \n"
"{                                                                            \n"
"   return sign(x) * pow(abs(x), 1.0 / 3.0);                                \n"
"}                                                                            \n"
"                                                                            \n"
"void main()                                                                \n"
"{                                                                            \n"
"   float d = q * q / 4.0 + p * p * p / 27.0;                                \n"
"                                                                            \n"
"   if (d >= 0.0)                                                            \n"
"   {                                                                        \n"
"      float c1 = -q / 2.0;                                                    \n"
"      float c2 = sqrt(d);                                                    \n"
"      if (check(cbrt(c1 + c2) + cbrt(c1 - c2) + offset)) discard;            \n"
"   }                                                                        \n"
"   else                                                                    \n"
"   {                                                                        \n"
"      float cos_3_theta = 3.0 * q * sqrt(-3.0 / p) / (2.0 * p);            \n"
"      float theta = acos(cos_3_theta) / 3.0;                                \n"
"      float r = 2.0 * sqrt(-p / 3.0);                                        \n"
"                                                                            \n"
"      if (check(r * cos(theta) + offset) &&                                \n"
"          check(r * cos(theta + 2.0 * M_PI / 3.0) + offset) &&                \n"
"          check(r * cos(theta + 4.0 * M_PI / 3.0) + offset)) discard;        \n"
"   }                                                                        \n"
"  gl_FragColor = vec4(1, 1, 1, 1);                                         \n"
"}                                                                            \n";



static const char *vs_code3 =
"attribute vec2 data0;                        \n"
"                                            \n"
"varying vec2 uv;                            \n"
"                                            \n"
"void main(void)                            \n"
"{                                            \n"
"  gl_Position = ftransform();                \n"
"  uv = data0;                                \n"
"}                                            \n";

static const char *fs_code3 =
"varying vec2 uv;                           \n"
"                                           \n"
"void main(void)                            \n"
"{                                          \n"
"  float u = uv.x;                          \n"
"  float v = uv.y;                          \n"
"  if (u * u + v * v > 1)                   \n"
"    discard;                               \n"
"  gl_FragColor = vec4(1, 1, 1, 1);         \n"
"}                                          \n";


static GLuint program0, program1, program2, program3;
static GLint matrix0, matrix1, matrix2, matrix3;
static GLfloat identity_matrix[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
#ifdef OPENGL_ES
static GLint mvp0, mvp1, mvp2, mvp3;
static GLfloat g_mvp[16];
#endif

#define DATA0_POS 0
#define DATA1_POS 1
#define DATA2_POS 2

static void subdivide_cubic(const double c[8], double c1[8], double c2[8])
{
    double p1x = (c[0] + c[2]) / 2;
    double p1y = (c[1] + c[3]) / 2;
    double p2x = (c[2] + c[4]) / 2;
    double p2y = (c[3] + c[5]) / 2;
    double p3x = (c[4] + c[6]) / 2;
    double p3y = (c[5] + c[7]) / 2;
    double p4x = (p1x + p2x) / 2;
    double p4y = (p1y + p2y) / 2;
    double p5x = (p2x + p3x) / 2;
    double p5y = (p2y + p3y) / 2;
    double p6x = (p4x + p5x) / 2;
    double p6y = (p4y + p5y) / 2;

    double p0x = c[0];
    double p0y = c[1];
    double p7x = c[6];
    double p7y = c[7];

    c1[0] = p0x;
    c1[1] = p0y;
    c1[2] = p1x;
    c1[3] = p1y;
    c1[4] = p4x;
    c1[5] = p4y;
    c1[6] = p6x;
    c1[7] = p6y;

    c2[0] = p6x;
    c2[1] = p6y;
    c2[2] = p5x;
    c2[3] = p5y;
    c2[4] = p3x;
    c2[5] = p3y;
    c2[6] = p7x;
    c2[7] = p7y;
}

static void subdivide_cubic2(const double cin[8], double cout[16])
{
    subdivide_cubic(cin, cout, cout + 8);
}

static void subdivide_cubic4(const double cin[8], double cout[32])
{
    subdivide_cubic(cin, cout, cout + 16);
    subdivide_cubic2(cout, cout);
    subdivide_cubic2(cout + 16, cout + 16);
}

static void subdivide_cubic8(const double cin[8], double cout[64])
{
    subdivide_cubic(cin, cout, cout + 32);
    subdivide_cubic4(cout, cout);
    subdivide_cubic4(cout + 32, cout + 32);
}

static void cubic_to_quadratic(const double c[8], double q[6])
{
    q[0] = c[0];
    q[1] = c[1];
    q[2] = (3 * (c[2] + c[4]) - (c[0] + c[6])) / 4;
    q[3] = (3 * (c[3] + c[5]) - (c[1] + c[7])) / 4;
    q[4] = c[6];
    q[5] = c[7];
}

/*
M: 0
L: 1
Z: 2

M -> M no
M -> L no
M -> Z no
L -> M yes
L -> L no
L -> Z no
Z -> M yes
Z -> L yes
Z -> Z no
*/
static const int new_path_table[3][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}};

// always starts with 'M'
// after 'M', only contains 'L' and 'Q'
// optionally finishes with 'Z'
struct reduced_path
{
    kvec_t(unsigned char) commands;
    kvec_t(float) coords;
};

typedef kvec_t(struct reduced_path) reduced_path_vec;

struct geometry
{
    kvec_t(float) vertices;
    kvec_t(unsigned short) indices;

    GLuint vertex_buffer;
    GLuint index_buffer;
    GLsizei count;
};

struct path
{
    int num_commands;
    unsigned char *commands;
    
    int num_coords;
    float *coords;
    
    reduced_path_vec reduced_paths;

    float stroke_width;
    int join_style;
    int initial_end_cap;
    int terminal_end_cap;
    float miter_limit;

    int num_dashes;
    float *dashes;
    float dash_length;

    struct geometry fill_geoms[4];      /* 0: front-solid, 1:back-solid 2:front-quad 3:back-quad */
    struct geometry stroke_geoms[2];    /* 1: solid 2: quad */

    GLuint fill_vertex_buffer;
    GLuint fill_index_buffer;
    GLsizei fill_counts[2];
    GLint fill_starts[2];

    float fill_bounds[4];
    float stroke_bounds[4];

    int is_stroke_dirty;
    int is_fill_dirty;
    int is_reduced_paths_dirty;
};

KHASH_MAP_INIT_INT(path, struct path *)

static khash_t(path) *paths = NULL;

static void delete_path(struct path *p)
{
    size_t i;

    free(p->commands);
    free(p->coords);

    free(p->dashes);

    for (i = 0; i < kv_size(p->reduced_paths); ++i)
    {
        kv_free(kv_a(p->reduced_paths, i).commands);
        kv_free(kv_a(p->reduced_paths, i).coords);
    }
    kv_free(p->reduced_paths);

#if 0
    for (i = 0; i < 4; ++i)
    {
        glDeleteBuffers(1, &p->fill_geoms[i].vertex_buffer);
        glDeleteBuffers(1, &p->fill_geoms[i].index_buffer);
    }
#endif

    for (i = 0; i < 2; ++i)
    {
        glDeleteBuffers(1, &p->stroke_geoms[i].vertex_buffer);
        glDeleteBuffers(1, &p->stroke_geoms[i].index_buffer);
    }

    glDeleteBuffers(1, &p->fill_vertex_buffer);
    glDeleteBuffers(1, &p->fill_index_buffer);

    free(p);
}

static void delete_path_helper(unsigned int min, unsigned int max)
{
    unsigned int i;
    for (i = min; i <= max; ++i)
    {
        khiter_t iter = kh_get(path, paths, i);
        if (iter != kh_end(paths))
        {
            struct path *p = kh_value(paths, iter);
            delete_path(p);
            kh_del(path, paths, iter);
        }
    }
}

static void new_path(reduced_path_vec *paths)
{
    struct reduced_path rp = {0};
    kv_push_back(*paths, rp);
}

static void move_to(struct reduced_path *path, float x, float y)
{
    if (kv_empty(path->commands))
    {
        kv_push_back(path->commands, GL_MOVE_TO_AC);
        kv_push_back(path->coords, x);
        kv_push_back(path->coords, y);
    }
    else
    {
        kv_a(path->coords, 0) = x;
        kv_a(path->coords, 1) = y;
    }
}

static void line_to(struct reduced_path *path, float x1, float y1, float x2, float y2)
{
    if (kv_empty(path->commands))
    {
        kv_push_back(path->commands, GL_MOVE_TO_AC);
        kv_push_back(path->coords, x1);
        kv_push_back(path->coords, y1);
    }

    kv_push_back(path->commands, GL_LINE_TO_AC);
    kv_push_back(path->coords, x2);
    kv_push_back(path->coords, y2);
}

static void quad_to(struct reduced_path *path, float x1, float y1, float x2, float y2, float x3, float y3)
{
    if (kv_empty(path->commands))
    {
        kv_push_back(path->commands, GL_MOVE_TO_AC);
        kv_push_back(path->coords, x1);
        kv_push_back(path->coords, y1);
    }

    kv_push_back(path->commands, GL_QUADRATIC_CURVE_TO_AC);
    kv_push_back(path->coords, x2);
    kv_push_back(path->coords, y2);
    kv_push_back(path->coords, x3);
    kv_push_back(path->coords, y3);
}

static void cubic_to(struct reduced_path *path, float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4)
{
    int i;

    double cin[8] = { x1, y1, x2, y2, x3, y3, x4, y4 };
    double cout[64];
    subdivide_cubic8(cin, cout);

    if (kv_empty(path->commands))
    {
        kv_push_back(path->commands, GL_MOVE_TO_AC);
        kv_push_back(path->coords, x1);
        kv_push_back(path->coords, y1);
    }

    for (i = 0; i < 8; ++i)
    {
        double q[6];
        cubic_to_quadratic(cout + i * 8, q);
        kv_push_back(path->commands, GL_QUADRATIC_CURVE_TO_AC);
        kv_push_back(path->coords, q[2]);
        kv_push_back(path->coords, q[3]);
        kv_push_back(path->coords, q[4]);
        kv_push_back(path->coords, q[5]);
    }
}

static double angle(double ux, double uy, double vx, double vy)
{
    return atan2(ux * vy - uy * vx, ux * vx + uy * vy);
}

/* http://www.w3.org/TR/SVG/implnote.html#ArcConversionEndpointToCenter */
static void endpoint_to_center(double x1, double y1, double x2, double y2,
                               int fA, int fS, double *prx, double *pry, double phi,
                               double *cx, double *cy, double *theta1, double *dtheta)
{
    double x1p, y1p, rx, ry, lambda, fsgn, c1, cxp, cyp;

    x1p =  cos(phi) * (x1 - x2) / 2 + sin(phi) * (y1 - y2) / 2;
    y1p = -sin(phi) * (x1 - x2) / 2 + cos(phi) * (y1 - y2) / 2;

    rx = *prx;
    ry = *pry;

    lambda = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
    if (lambda > 1)
    {
        lambda = sqrt(lambda);
        rx *= lambda;
        ry *= lambda;
        *prx = rx;
        *pry = ry;
    }

    fA = !!fA;
    fS = !!fS;

    fsgn = (fA != fS) ? 1 : -1;

    c1 = (rx*rx*ry*ry - rx*rx*y1p*y1p - ry*ry*x1p*x1p) / (rx*rx*y1p*y1p + ry*ry*x1p*x1p);

    if (c1 < 0)	// because of floating point inaccuracies, c1 can be -epsilon.
        c1 = 0;
    else
        c1 = sqrt(c1);

    cxp = fsgn * c1 * (rx * y1p / ry);
    cyp = fsgn * c1 * (-ry * x1p / rx);

    *cx = cos(phi) * cxp - sin(phi) * cyp + (x1 + x2) / 2;
    *cy = sin(phi) * cxp + cos(phi) * cyp + (y1 + y2) / 2;

    *theta1 = angle(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry);

    *dtheta = angle((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry);

    if (!fS && *dtheta > 0)
        *dtheta -= 2 * M_PI;
    else if (fS && *dtheta < 0)
        *dtheta += 2 * M_PI;
}

static void arc_tod(struct reduced_path *path, double x1, double y1, double rh, double rv, double phi, int fA, int fS, double x2, double y2)
{
    double cx, cy, theta1, dtheta;

    int i, nquads;

    phi *= M_PI / 180;

    endpoint_to_center(x1, y1, x2, y2, fA, fS, &rh, &rv, phi, &cx, &cy, &theta1, &dtheta);

    nquads = ceil(fabs(dtheta) * 4 / M_PI);

    for (i = 0; i < nquads; ++i)
    {
        double t1 = theta1 + (i / (double)nquads) * dtheta;
        double t2 = theta1 + ((i + 1) / (double)nquads) * dtheta;
        double tm = (t1 + t2) / 2;

        double x1 = cos(phi)*rh*cos(t1) - sin(phi)*rv*sin(t1) + cx;
        double y1 = sin(phi)*rh*cos(t1) + cos(phi)*rv*sin(t1) + cy;

        double x2 = cos(phi)*rh*cos(t2) - sin(phi)*rv*sin(t2) + cx;
        double y2 = sin(phi)*rh*cos(t2) + cos(phi)*rv*sin(t2) + cy;

        double xm = cos(phi)*rh*cos(tm) - sin(phi)*rv*sin(tm) + cx;
        double ym = sin(phi)*rh*cos(tm) + cos(phi)*rv*sin(tm) + cy;

        double xc = (xm * 4 - (x1 + x2)) / 2;
        double yc = (ym * 4 - (y1 + y2)) / 2;

        kv_push_back(path->commands, GL_QUADRATIC_CURVE_TO_AC);
        kv_push_back(path->coords, xc);
        kv_push_back(path->coords, yc);
        kv_push_back(path->coords, x2);
        kv_push_back(path->coords, y2);
    }
}

static void arc_to(struct reduced_path *path, float x1, float y1, float rh, float rv, float phi, int fA, int fS, float x2, float y2)
{
    if (kv_empty(path->commands))
    {
        kv_push_back(path->commands, GL_MOVE_TO_AC);
        kv_push_back(path->coords, x1);
        kv_push_back(path->coords, y1);
    }

    arc_tod(path, x1, y1, rh, rv, phi, fA, fS, x2, y2);
}

static void close_path(struct reduced_path *path)
{
    if (kv_back(path->commands) != GL_CLOSE_PATH_AC)
        kv_push_back(path->commands, GL_CLOSE_PATH_AC);
}

static void reduce_path(int num_commands, const unsigned char *commands, int num_coords, const float *coords, reduced_path_vec *reduced_paths)
{
#define c0 coords[icoord]
#define c1 coords[icoord + 1]
#define c2 coords[icoord + 2]
#define c3 coords[icoord + 3]
#define c4 coords[icoord + 4]
#define c5 coords[icoord + 5]
#define c6 coords[icoord + 6]

#define set(x1, y1, x2, y2) ncpx = x1; ncpy = y1; npepx = x2; npepy = y2;

#define last_path &kv_back(*reduced_paths)


    int icoord = 0;

    float spx = 0, spy = 0;
    float cpx = 0, cpy = 0;
    float pepx = 0, pepy = 0;
    float ncpx = 0, ncpy = 0;
    float npepx = 0, npepy = 0;

    unsigned char prev_command = 2;

    int i;

    for (i = 0; i < num_commands; ++i)
    {
        switch(commands[i])
        {
        case GL_MOVE_TO_AC:
        case 'M':
            if (new_path_table[prev_command][0])
                new_path(reduced_paths);
            prev_command = 0;
            move_to(last_path, c0, c1);
            set(c0, c1, c0, c1);
            spx = ncpx;
            spy = ncpy;
            icoord += 2;
            break;

        case GL_RELATIVE_MOVE_TO_AC:
        case 'm':
            if (new_path_table[prev_command][0])
                new_path(reduced_paths);
            prev_command = 0;
            move_to(last_path, cpx + c0, cpy + c1);
            set(cpx + c0, cpy + c1, cpx + c0, cpy + c1);
            spx = ncpx;
            spy = ncpy;
            icoord += 2;
            break;

        case GL_CLOSE_PATH_AC:
        case 'z':
        case 'Z':
            if (new_path_table[prev_command][2])
                new_path(reduced_paths);
            prev_command = 2;
            close_path(last_path);
            set(spx, spy, spx, spy);
            break;

        case GL_RESTART_PATH_AC:
            if (new_path_table[prev_command][0])
                new_path(reduced_paths);
            prev_command = 0;
            move_to(last_path, 0, 0);
            set(0, 0, 0, 0);
            spx = ncpx;
            spy = ncpy;
            break;

        case GL_LINE_TO_AC:
        case 'L':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, c0, c1);
            set(c0, c1, c0, c1);
            icoord += 2;
            break;
            
        case GL_RELATIVE_LINE_TO_AC:
        case 'l':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, cpx + c0, cpy + c1);
            set(cpx + c0, cpy + c1, cpx + c0, cpy + c1);
            icoord += 2;
            break;

        case GL_HORIZONTAL_LINE_TO_AC:
        case 'H':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, c0, cpy);
            set(c0, cpy, c0, cpy);
            icoord += 1;
            break;

        case GL_RELATIVE_HORIZONTAL_LINE_TO_AC:
        case 'h':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, cpx + c0, cpy);
            set(cpx + c0, cpy, cpx + c0, cpy);
            icoord += 1;
            break;

        case GL_VERTICAL_LINE_TO_AC:
        case 'V':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, cpx, c0);
            set(cpx, c0, cpx, c0);
            icoord += 1;
            break;

        case GL_RELATIVE_VERTICAL_LINE_TO_AC:
        case 'v':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            line_to(last_path, cpx, cpy, cpx, cpy + c0);
            set(cpx, cpy + c0, cpx, cpy + c0);
            icoord += 1;
            break;

        case GL_QUADRATIC_CURVE_TO_AC:
        case 'Q':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            quad_to(last_path, cpx, cpy, c0, c1, c2, c3);
            set(c2, c3, c0, c1);
            icoord += 4;
            break;

        case GL_RELATIVE_QUADRATIC_CURVE_TO_AC:
        case 'q':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            quad_to(last_path, cpx, cpy, cpx + c0, cpy + c1, cpx + c2, cpy + c3);
            set(cpx + c2, cpy + c3, cpx + c0, cpy + c1);
            icoord += 4;
            break;

        case GL_CUBIC_CURVE_TO_AC:
        case 'C':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, c0, c1, c2, c3, c4, c5);
            set(c4, c5, c2, c3);
            icoord += 6;
            break;

        case GL_RELATIVE_CUBIC_CURVE_TO_AC:
        case 'c':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, cpx + c0, cpy + c1, cpx + c2, cpy + c3, cpx + c4, cpy + c5);
            set(cpx + c4, cpy + c5, cpx + c2, cpy + c3);
            icoord += 6;
            break;

        case GL_SMOOTH_QUADRATIC_CURVE_TO_AC:
        case 'T':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            quad_to(last_path, cpx, cpy, 2 * cpx - pepx, 2 * cpy - pepy, c0, c1);
            set(c0, c1, 2 * cpx - pepx, 2 * cpy - pepy);
            icoord += 2;
            break;
            
        case GL_RELATIVE_SMOOTH_QUADRATIC_CURVE_TO_AC:
        case 't':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            quad_to(last_path, cpx, cpy, 2 * cpx - pepx, 2 * cpy - pepy, cpx + c0, cpy + c1);
            set(cpx + c0, cpy + c1, 2 * cpx - pepx, 2 * cpy - pepy);
            icoord += 2;
            break;

        case GL_SMOOTH_CUBIC_CURVE_TO_AC:
        case 'S':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, 2 * cpx - pepx, 2 * cpy - pepy, c0, c1, c2, c3);
            set(c2, c3, c0, c1);
            icoord += 4;
            break;

        case GL_RELATIVE_SMOOTH_CUBIC_CURVE_TO_AC:
        case 's':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, 2 * cpx - pepx, 2 * cpy - pepy, cpx + c0, cpy + c1, cpx + c2, cpy + c3);
            set(cpx + c2, cpy + c3, cpx + c0, cpy + c1);
            icoord += 4;
            break;

        case GL_SMALL_CCW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 0, 1, c3, c4);
            set(c3, c4, c3, c4);
            icoord += 5;
            break;
            
        case GL_RELATIVE_SMALL_CCW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 0, 1, cpx + c3, cpy + c4);
            set(cpx + c3, cpy + c4, cpx + c3, cpy + c4);
            icoord += 5;
            break;

        case GL_SMALL_CW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 0, 0, c3, c4);
            set(c3, c4, c3, c4);
            icoord += 5;
            break;
            
        case GL_RELATIVE_SMALL_CW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 0, 0, cpx + c3, cpy + c4);
            set(cpx + c3, cpy + c4, cpx + c3, cpy + c4);
            icoord += 5;
            break;

        case GL_LARGE_CCW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 1, 1, c3, c4);
            set(c3, c4, c3, c4);
            icoord += 5;
            break;
            
        case GL_RELATIVE_LARGE_CCW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 1, 1, cpx + c3, cpy + c4);
            set(cpx + c3, cpy + c4, cpx + c3, cpy + c4);
            icoord += 5;
            break;

        case GL_LARGE_CW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 1, 0, c3, c4);
            set(c3, c4, c3, c4);
            icoord += 5;
            break;
            
        case GL_RELATIVE_LARGE_CW_ARC_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, 1, 0, cpx + c3, cpy + c4);
            set(cpx + c3, cpy + c4, cpx + c3, cpy + c4);
            icoord += 5;
            break;

        case GL_ARC_TO_AC:
        case 'A':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, c3, c4, c5, c6);
            set(c5, c6, c5, c6);
            icoord += 7;
            break;

        case GL_RELATIVE_ARC_TO_AC:
        case 'a':
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            arc_to(last_path, cpx, cpy, c0, c1, c2, c3, c4, cpx + c5, cpy + c6);
            set(cpx + c5, cpy + c6, cpx + c5, cpy + c6);
            icoord += 7;
            break;

        case GL_RECT_AC:
            if (new_path_table[prev_command][0])
                new_path(reduced_paths);
            prev_command = 2;
            move_to(last_path, c0, c1);
            line_to(last_path, c0, c1, c0 + c2, c1);
            line_to(last_path, c0 + c2, c1, c0 + c2, c1 + c3);
            line_to(last_path, c0 + c2, c1 + c3, c0, c1 + c3);
            close_path(last_path);
            set(c0, c1, c0, c1 + c3);
            icoord += 4;
            break;

        case GL_DUP_FIRST_CUBIC_CURVE_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, cpx, cpy, c0, c1, c2, c3);
            set(c2, c3, c0, c1);
            icoord += 4;
            break;

        case GL_DUP_LAST_CUBIC_CURVE_TO_AC:
            if (new_path_table[prev_command][1])
                new_path(reduced_paths);
            prev_command = 1;
            cubic_to(last_path, cpx, cpy, c0, c1, c2, c3, c2, c3);
            set(c2, c3, c2, c3);
            icoord += 4;
            break;

        case GL_CIRCULAR_CCW_ARC_TO_AC:
        case GL_CIRCULAR_CW_ARC_TO_AC:
        case GL_CIRCULAR_TANGENT_ARC_TO_AC:
            // todo
            break;
        }

        cpx = ncpx;
        cpy = ncpy;
        pepx = npepx;
        pepy = npepy;
    }

#undef c0
#undef c1
#undef c2
#undef c3
#undef c4
#undef c5
#undef c6

#undef set

#undef last_path
}

static int command_coords[256];

static void path_commands(unsigned int path, int num_commands, const unsigned char *commands, int num_coords, const float *coords)
{
    int i;
    int num_coords2;
    khiter_t iter;
    struct path *p;
    int tmp;

    num_coords2 = 0;
    for (i = 0; i < num_commands; ++i)
    {
        int num = command_coords[commands[i]];
        if (num == -1)
        {
            // TODO: set error
            return;
        }
        num_coords2 += num;
    }

    if (num_coords != num_coords2)
    {
        // TODO: set error
        return;
    }

    iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        iter = kh_put(path, paths, path, &tmp);
        p = (struct path*)malloc(sizeof(struct path));
        kh_value(paths, iter) = p;

        p->stroke_width = 1;
        p->join_style = GL_MITER_REVERT_AC;
        p->initial_end_cap = GL_FLAT;
        p->terminal_end_cap = GL_FLAT;
        p->miter_limit = 4;
        p->num_dashes = 0;
        p->dashes = NULL;
        p->dash_length = 0;

#if 0
        for (i = 0; i < 4; ++i)
        {
            glGenBuffers(1, &p->fill_geoms[i].vertex_buffer);
            glGenBuffers(1, &p->fill_geoms[i].index_buffer);
            p->fill_geoms[i].count = 0;
        }
#endif

        for (i = 0; i < 2; ++i)
        {
            glGenBuffers(1, &p->stroke_geoms[i].vertex_buffer);
            glGenBuffers(1, &p->stroke_geoms[i].index_buffer);
            p->stroke_geoms[i].count = 0;
        }

        glGenBuffers(1, &p->fill_vertex_buffer);
        glGenBuffers(1, &p->fill_index_buffer);

        set_path(path);
    }
    else
    {
        size_t i;

        p = kh_value(paths, iter);

        free(p->commands);
        free(p->coords);
        for (i = 0; i < kv_size(p->reduced_paths); ++i)
        {
            kv_free(kv_a(p->reduced_paths, i).commands);
            kv_free(kv_a(p->reduced_paths, i).coords);
        }
        kv_free(p->reduced_paths);
    }

    p->num_commands = num_commands;
    p->commands = (unsigned char*)malloc(num_commands * sizeof(unsigned char));
    memcpy(p->commands, commands, num_commands * sizeof(unsigned char));

    p->num_coords = num_coords;
    p->coords = (float *)malloc(num_coords * sizeof(float));
    memcpy(p->coords, coords, num_coords * sizeof(float));

    kv_init(p->reduced_paths);

    reduce_path(num_commands, commands, num_coords, coords, &p->reduced_paths);

    p->is_fill_dirty = 1;
    p->is_stroke_dirty = 1;
    p->is_reduced_paths_dirty = 0;
}

static void add_stroke_line(struct path *p, double x0, double y0, double x1, double y1)
{
    struct geometry *g = &p->stroke_geoms[0];

    double width = p->stroke_width;

    int index = kv_size(g->vertices) / 2;

    double dx = x1 - x0;
    double dy = y1 - y0;
    double len = sqrt(dx * dx + dy * dy);
    if (len == 0)
        return;

    dx /= len;
    dy /= len;

    kv_push_back(g->vertices, x0 + (dy * width * 0.5));
    kv_push_back(g->vertices, y0 - (dx * width * 0.5));

    kv_push_back(g->vertices, x1 + (dy * width * 0.5));
    kv_push_back(g->vertices, y1 - (dx * width * 0.5));

    kv_push_back(g->vertices, x1 - (dy * width * 0.5));
    kv_push_back(g->vertices, y1 + (dx * width * 0.5));

    kv_push_back(g->vertices, x0 - (dy * width * 0.5));
    kv_push_back(g->vertices, y0 + (dx * width * 0.5));

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 1);
    kv_push_back(g->indices, index + 2);

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 2);
    kv_push_back(g->indices, index + 3);
}

static void add_stroke_line_dashed(struct path *path, double x0, double y0, double x1, double y1, double *dash_offset)
{
    if (path->num_dashes == 0)
    {
        add_stroke_line(path, x0, y0, x1, y1);
        return;
    }

    double length = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

    /* TODO: remove this check and make sure that 0 length lines are not passed to this function (while creaating reduced path and while closing the path) */
    if (length == 0)
        return;

    double offset = -fmod(*dash_offset, path->dash_length);

    int i = 0;
    while (offset < length)
    {
        double o0 = MAX(offset, 0);
        double o1 = MIN(offset + path->dashes[i], length);

        if (o1 >= 0)
        {
            double t0 = o0 / length;
            double t1 = o1 / length;

            add_stroke_line(path, (1 - t0) * x0 + t0 * x1, (1 - t0) * y0 + t0 * y1, (1 - t1) * x0 + t1 * x1, (1 - t1) * y0 + t1 * y1);
        }

        offset += path->dashes[i] + path->dashes[i + 1];
        i = (i + 2) % path->num_dashes;
    }

    *dash_offset = fmod(length + *dash_offset, path->dash_length);
}

static void evaluate_quadratic(double x0, double y0, double x1, double y1, double x2, double y2, double t, double *x, double *y)
{
    double Ax = x0 - 2 * x1 + x2;
    double Ay = y0 - 2 * y1 + y2;
    double Bx = 2 * (x1 - x0);
    double By = 2 * (y1 - y0);
    double Cx = x0;
    double Cy = y0;

    *x = Ax * t * t + Bx * t + Cx;
    *y = Ay * t * t + By * t + Cy;
}

static void get_quadratic_bounds(double x0, double y0, double x1, double y1, double x2, double y2,
                                 double stroke_width,
                                 double *minx, double *miny, double *maxx, double *maxy)
{
    // TODO: if control point is exactly between start and end, division by zero occurs
    double tx = (x0 - x1) / (x0 - 2 * x1 + x2);
    double ty = (y0 - y1) / (y0 - 2 * y1 + y2);

    *minx = MIN(x0, x2);
    *miny = MIN(y0, y2);
    *maxx = MAX(x0, x2);
    *maxy = MAX(y0, y2);

    if (0 < tx && tx < 1)
    {
        double x, y;
        evaluate_quadratic(x0, y0, x1, y1, x2, y2, tx, &x, &y);
        
        *minx = MIN(*minx, x);
        *miny = MIN(*miny, y);
        *maxx = MAX(*maxx, x);
        *maxy = MAX(*maxy, y);
    }

    if (0 < ty && ty < 1)
    {
        double x, y;
        evaluate_quadratic(x0, y0, x1, y1, x2, y2, ty, &x, &y);

        *minx = MIN(*minx, x);
        *miny = MIN(*miny, y);
        *maxx = MAX(*maxx, x);
        *maxy = MAX(*maxy, y);
    }

    *minx -= stroke_width * 0.5;
    *miny -= stroke_width * 0.5;
    *maxx += stroke_width * 0.5;
    *maxy += stroke_width * 0.5;
}

static double dot(double x0, double y0, double x1, double y1)
{
    return x0 * x1 + y0 * y1;
}

static void get_quadratic_bounds_oriented(double x0, double y0, double x1, double y1, double x2, double y2,
                                          double stroke_width,
                                          double *pcx, double *pcy, double *pux, double *puy, double *pvx, double *pvy)
{
    double minx, miny, maxx, maxy;
    double cx, cy;
    double ex, ey;

    double ux = x2 - x0;
    double uy = y2 - y0;

    double len = sqrt(ux * ux + uy * uy);
    if (len < 1e-6)
    {
        ux = 1;
        uy = 0;
    }
    else
    {
        ux /= len;
        uy /= len;
    }

    get_quadratic_bounds(0, 0,
        dot(ux, uy, x1 - x0, y1 - y0), dot(-uy, ux, x1 - x0, y1 - y0),
        dot(ux, uy, x2 - x0, y2 - y0), dot(-uy, ux, x2 - x0, y2 - y0),
        stroke_width,
        &minx, &miny, &maxx, &maxy);

    cx = (minx + maxx) / 2;
    cy = (miny + maxy) / 2;
    ex = (maxx - minx) / 2;
    ey = (maxy - miny) / 2;

    *pcx = x0 + ux * cx + -uy * cy;
    *pcy = y0 + uy * cx +  ux * cy;
    *pux = ux * ex;
    *puy = uy * ex;
    *pvx = -uy * ey;
    *pvy = ux * ey;
}

static void calculatepq(double Ax, double Ay, double Bx, double By, double Cx, double Cy,
                        double px, double py,
                        double *p, double *q)
{
    double a = -2 * dot(Ax, Ay, Ax, Ay);
    double b = -3 * dot(Ax, Ay, Bx, By);
    double c = 2 * dot(px, py, Ax, Ay) - 2 * dot(Cx, Cy, Ax, Ay) - dot(Bx, By, Bx, By);
    double d = dot(px, py, Bx, By) - dot(Cx, Cy, Bx, By);

    *p = (3 * a * c - b * b) / (3 * a * a);
    *q = (2 * b * b * b - 9 * a * b * c + 27 * a * a * d) / (27 * a * a * a);
}


static double arc_length_helper(double A, double B, double C, double t)
{
    /* Integrate[Sqrt[A t^2 + B t + C], t] */
    return (2 * sqrt(A) * (B + 2 * A * t) * sqrt(C + t * (B + A * t)) - (B * B - 4 * A * C) * log(B + 2 * A * t + 2 * sqrt(A) * sqrt(C + t * (B + A * t)))) / (8 * sqrt(A * A * A));
}

static double arc_length(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double t)
{
    double A = 4 * (Ax * Ax + Ay * Ay);
    double B = 4 * (Ax * Bx + Ay * By);
    double C = Bx * Bx + By * By;

    return arc_length_helper(A, B, C, t) - arc_length_helper(A, B, C, 0);
}

static double inverse_arc_length(double Ax, double Ay, double Bx, double By, double Cx, double Cy, double u)
{
    int i;

    double A = 4 * (Ax * Ax + Ay * Ay);
    double B = 4 * (Ax * Bx + Ay * By);
    double C = Bx * Bx + By * By;

    if (u <= 0)
        return 0;

    double length = arc_length_helper(A, B, C, 1) - arc_length_helper(A, B, C, 0);

    if (u >= length)
        return 1;

    double u0 = arc_length_helper(A, B, C, 0) + u;

    double a = 0;
    double b = 1;
    double fa = arc_length_helper(A, B, C, a) - u0;
    double fb = arc_length_helper(A, B, C, b) - u0;

    for (i = 0; i < 20; ++i)
    {
        double c = (a + b) / 2;
        double fc = arc_length_helper(A, B, C, c) - u0;

        if (fc < 0)
        {
            a = c;
            fa = fc;
        }
        else if (fc > 0)
        {
            b = c;
            fb = fc;
        }
        else
            break;
    }

    return a - fa * (b - a) / (fb - fa);
}

static void quad_segment(const double qin[6], double t0, double t1, double qout[6])
{
    double u0 = 1 - t0;
    double u1 = 1 - t1;

    double x0 = qin[0];
    double y0 = qin[1];
    double x1 = qin[2];
    double y1 = qin[3];
    double x2 = qin[4];
    double y2 = qin[5];

    qout[0] = (u0 * u0) * x0 + (u0 * t0 + u0 * t0) * x1 + (t0 * t0) * x2;
    qout[1] = (u0 * u0) * y0 + (u0 * t0 + u0 * t0) * y1 + (t0 * t0) * y2;
    qout[2] = (u0 * u1) * x0 + (u0 * t1 + u1 * t0) * x1 + (t0 * t1) * x2;
    qout[3] = (u0 * u1) * y0 + (u0 * t1 + u1 * t0) * y1 + (t0 * t1) * y2;
    qout[4] = (u1 * u1) * x0 + (u1 * t1 + u1 * t1) * x1 + (t1 * t1) * x2;
    qout[5] = (u1 * u1) * y0 + (u1 * t1 + u1 * t1) * y1 + (t1 * t1) * y2;
}


static void add_stroke_quad(struct path *path, double x0, double y0, double x1, double y1, double x2, double y2)
{
    int i;

    double Ax = x0 - 2 * x1 + x2;
    double Ay = y0 - 2 * y1 + y2;
    double Bx = 2 * (x1 - x0);
    double By = 2 * (y1 - y0);
    double Cx = x0;
    double Cy = y0;

    double cx, cy, ux, uy, vx, vy;
    get_quadratic_bounds_oriented(x0, y0, x1, y1, x2, y2, path->stroke_width + 1, &cx, &cy, &ux, &uy, &vx, &vy);

    double a = -2 * dot(Ax, Ay, Ax, Ay);
    double b = -3 * dot(Ax, Ay, Bx, By);

    double px[4], py[4];

    px[0] = cx - ux - vx;
    py[0] = cy - uy - vy;
    px[1] = cx + ux - vx;
    py[1] = cy + uy - vy;
    px[2] = cx + ux + vx;
    py[2] = cy + uy + vy;
    px[3] = cx - ux + vx;
    py[3] = cy - uy + vy;

    double p[4], q[4];
    for (i = 0; i < 4; ++i)
        calculatepq(Ax, Ay, Bx, By, Cx, Cy, px[i], py[i], &p[i], &q[i]);

    struct geometry *g = &path->stroke_geoms[1];

    int index = kv_size(g->vertices) / 12;

    for (i = 0; i < 4; ++i)
    {
        kv_push_back(g->vertices, px[i]);
        kv_push_back(g->vertices, py[i]);
        kv_push_back(g->vertices, p[i]);
        kv_push_back(g->vertices, q[i]);
        kv_push_back(g->vertices, Ax);
        kv_push_back(g->vertices, Ay);
        kv_push_back(g->vertices, Bx);
        kv_push_back(g->vertices, By);
        kv_push_back(g->vertices, Cx);
        kv_push_back(g->vertices, Cy);
        kv_push_back(g->vertices, -b / (3 * a));
        kv_push_back(g->vertices, path->stroke_width * path->stroke_width / 4);
    }

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 1);
    kv_push_back(g->indices, index + 2);

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 2);
    kv_push_back(g->indices, index + 3);
}

static void add_stroke_quad_dashed(struct path *path, double x0, double y0, double x1, double y1, double x2, double y2, double *dash_offset)
{
    if (path->num_dashes == 0)
    {
        add_stroke_quad(path, x0, y0, x1, y1, x2, y2);
        return;
    }

    double Ax = x0 - 2 * x1 + x2;
    double Ay = y0 - 2 * y1 + y2;
    double Bx = 2 * (x1 - x0);
    double By = 2 * (y1 - y0);
    double Cx = x0;
    double Cy = y0;

    double q[6] = { x0, y0, x1, y1, x2, y2 };

    double length = arc_length(Ax, Ay, Bx, By, Cx, Cy, 1);

    double offset = -fmod(*dash_offset, path->dash_length);

    int i = 0;
    while (offset < length)
    {
        double o0 = MAX(offset, 0);
        double o1 = MIN(offset + path->dashes[i], length);

        if (o1 >= 0)
        {
            double t0 = inverse_arc_length(Ax, Ay, Bx, By, Cx, Cy, o0);
            double t1 = inverse_arc_length(Ax, Ay, Bx, By, Cx, Cy, o1);

            double qout[6];
            quad_segment(q, t0, t1, qout);
            add_stroke_quad(path, qout[0], qout[1], qout[2], qout[3], qout[4], qout[5]);
        }

        offset += path->dashes[i] + path->dashes[i + 1];
        i = (i + 2) % path->num_dashes;
    }

    *dash_offset = fmod(length + *dash_offset, path->dash_length);
}


static void add_join_miter_revert(struct path *path, float x0, float y0, float x1, float y1, float x2, float y2)
{

}

static void add_join_miter_truncate(struct path *path, float x0, float y0, float x1, float y1, float x2, float y2)
{

}

static void add_join_bevel(struct path *path, float x0, float y0, float x1, float y1, float x2, float y2)
{
    float v0x = x0 - x1;
    float v0y = y0 - y1;
    float v1x = x2 - x1;
    float v1y = y2 - y1;

    float len0 = sqrtf(v0x * v0x + v0y * v0y);
    float len1 = sqrtf(v1x * v1x + v1y * v1y);

    if (len0 == 0 || len1 == 0)
        return;

    struct geometry *g = &path->stroke_geoms[0];

    float width = path->stroke_width;

    int index = kv_size(g->vertices) / 2;

    float w0 = width / (2 * len0);
    float w1 = width / (2 * len1);

    if (v0x * v1y - v0y * v1x < 0)
    {
        kv_push_back(g->vertices, x1 + v1y * w1);
        kv_push_back(g->vertices, y1 + -v1x * w1);
        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, x1 + -v0y * w0);
        kv_push_back(g->vertices, y1 + v0x * w0);
    }
    else
    {
        kv_push_back(g->vertices, x1 + v0y * w0);
        kv_push_back(g->vertices, y1 + -v0x * w0);
        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, x1 + -v1y * w1);
        kv_push_back(g->vertices, y1 + v1x * w1);
    }

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 1);
    kv_push_back(g->indices, index + 2);
}

static void add_join_round(struct path *path, float x0, float y0, float x1, float y1, float x2, float y2)
{

}

static int check_offset(struct path *path, float of)
{
    int i;

    if (path->num_dashes == 0)
        return 1;

    float offset = 0;
    for (i = 0; i < path->num_dashes; i += 2)
    {
        float o0 = offset;
        float o1 = offset + path->dashes[i];

        if (o0 <= of && of <= o1)
            return 1;

        offset += path->dashes[i] + path->dashes[i + 1];
    }

    return 0;
}

static void add_join(struct path *path, float x0, float y0, float x1, float y1, float x2, float y2)
{
    switch (path->join_style)
    {
    case GL_MITER_REVERT_AC:
        add_join_miter_revert(path, x0, y0, x1, y1, x2, y2);
        break;
    case GL_MITER_TRUNCATE_AC:
        add_join_miter_truncate(path, x0, y0, x1, y1, x2, y2);
        break;
    case GL_BEVEL_AC:
        add_join_bevel(path, x0, y0, x1, y1, x2, y2);
        break;
    case GL_ROUND_AC:
        add_join_round(path, x0, y0, x1, y1, x2, y2);
        break;
    case GL_NONE:
        break;
    }
}

typedef kvec_t(float) kvec_float_t;

static void corner_start(kvec_float_t *v, float x, float y, float offset)
{
    kv_push_back(*v, x);
    kv_push_back(*v, y);
    kv_push_back(*v, offset);
}

static void corner_continue(kvec_float_t *v, float x0, float y0, float x1, float y1, float offset)
{
    float x = kv_a(*v, kv_size(*v) - 3);
    float y = kv_a(*v, kv_size(*v) - 2);

    // TODO: check equality with epsilon
    if (x != x0 || x != x1 || x0 != x1 || y != y0 || y != y1 || y0 != y1)
    {
        kv_push_back(*v, x0);
        kv_push_back(*v, y0);
        kv_push_back(*v, x1);
        kv_push_back(*v, y1);
        kv_push_back(*v, offset);
    }
}

static void corner_end(kvec_float_t *v, float x, float y)
{
    float x0 = kv_a(*v, 0);
    float y0 = kv_a(*v, 1);

    float x1 = kv_a(*v, kv_size(*v) - 3);
    float y1 = kv_a(*v, kv_size(*v) - 2);

    // TODO: check equality with epsilon
    if (x != x0 || x != x1 || x0 != x1 || y != y0 || y != y1 || y0 != y1)
    {
        kv_push_back(*v, x);
        kv_push_back(*v, y);
    }
    else
    {
        kv_pop_back(*v);
        kv_pop_back(*v);
        kv_pop_back(*v);
    }
}

static void update_bounds(float bounds[4], size_t num_vertices, const float *vertices, int stride)
{
    size_t i;

    for (i = 0; i < num_vertices; i += stride)
    {
        float x = vertices[i];
        float y = vertices[i + 1];

        bounds[0] = MIN(bounds[0], x);
        bounds[1] = MIN(bounds[1], y);
        bounds[2] = MAX(bounds[2], x);
        bounds[3] = MAX(bounds[3], y);
    }
}

static void create_stroke_geometry(struct path *path)
{
#define c0 coords[icoord]
#define c1 coords[icoord + 1]
#define c2 coords[icoord + 2]
#define c3 coords[icoord + 3]

#define set(x1, y1, x2, y2) ncpx = x1; ncpy = y1; npepx = x2; npepy = y2;

    reduced_path_vec *reduced_paths = &path->reduced_paths;

    size_t i, j;

    for (i = 0; i < 2; ++i)
    {
        kv_init(path->stroke_geoms[i].vertices);
        kv_init(path->stroke_geoms[i].indices);
    }

    double offset = 0;

    for (i = 0; i < kv_size(*reduced_paths); ++i)
    {
        struct reduced_path *p = &kv_a(*reduced_paths, i);

        kvec_float_t corners;
        kv_init(corners);

        size_t num_commands = kv_size(p->commands);
        unsigned char *commands = kv_data(p->commands);
        float *coords = kv_data(p->coords);

        int closed = 0;

        int icoord = 0;

        float spx = 0, spy = 0;
        float cpx = 0, cpy = 0;
        float pepx = 0, pepy = 0;
        float ncpx = 0, ncpy = 0;
        float npepx = 0, npepy = 0;

        for (j = 0; j < num_commands; ++j)
        {
            switch (commands[j])
            {
            case GL_MOVE_TO_AC:
                corner_start(&corners, c0, c1, offset);
                set(c0, c1, c0, c1);
                spx = ncpx;
                spy = ncpy;
                icoord += 2;
                break;
            case GL_LINE_TO_AC:
                add_stroke_line_dashed(path, cpx, cpy, c0, c1, &offset);
                corner_continue(&corners, (cpx + c0) / 2, (cpy + c1) / 2, c0, c1, offset);
                set(c0, c1, c0, c1);
                icoord += 2;
                break;
            case GL_QUADRATIC_CURVE_TO_AC:
                add_stroke_quad_dashed(path, cpx, cpy, c0, c1, c2, c3, &offset);
                corner_continue(&corners, c0, c1, c2, c3, offset);
                set(c2, c3, c0, c1);
                icoord += 4;
                break;
            case GL_CLOSE_PATH_AC:
                add_stroke_line_dashed(path, cpx, cpy, spx, spy, &offset);
                corner_end(&corners, (cpx + spx) / 2, (cpy + spy) / 2);
                set(spx, spy, spx, spy);
                closed = 1;
                break;
            }

            cpx = ncpx;
            cpy = ncpy;
            pepx = npepx;
            pepy = npepy;
        }

        size_t ncorners = (kv_size(corners) - (closed ? 0 : 7)) / 5;

        for (j = 0; j < ncorners; j++)
        {
            int j0 = j;
            int j1 = (j + 1) % ncorners;

            float x0 = kv_a(corners, j0 * 5 + 3);
            float y0 = kv_a(corners, j0 * 5 + 4);
            float x1 = kv_a(corners, j1 * 5 + 0);
            float y1 = kv_a(corners, j1 * 5 + 1);
            float of = kv_a(corners, j1 * 5 + 2);
            float x2 = kv_a(corners, j1 * 5 + 3);
            float y2 = kv_a(corners, j1 * 5 + 4);

            if (check_offset(path, of))
                add_join(path, x0, y0, x1, y1, x2, y2);
        }

        kv_free(corners);
    }

#undef c0
#undef c1
#undef c2
#undef c3

#undef set

    path->stroke_bounds[0] = 1e30f;
    path->stroke_bounds[1] = 1e30f;
    path->stroke_bounds[2] = -1e30f;
    path->stroke_bounds[3] = -1e30f;

    update_bounds(path->stroke_bounds, kv_size(path->stroke_geoms[0].vertices), kv_data(path->stroke_geoms[0].vertices), 2);
    update_bounds(path->stroke_bounds, kv_size(path->stroke_geoms[1].vertices), kv_data(path->stroke_geoms[1].vertices), 12);

    for (i = 0; i < 2; ++i)
    {
        glBindBuffer(GL_ARRAY_BUFFER, path->stroke_geoms[i].vertex_buffer);
        glBufferData(GL_ARRAY_BUFFER, kv_size(path->stroke_geoms[i].vertices) * sizeof(float), kv_data(path->stroke_geoms[i].vertices), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, path->stroke_geoms[i].index_buffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, kv_size(path->stroke_geoms[i].indices) * sizeof(unsigned short), kv_data(path->stroke_geoms[i].indices), GL_STATIC_DRAW);

        path->stroke_geoms[i].count = kv_size(path->stroke_geoms[i].indices);

        kv_free(path->stroke_geoms[i].vertices);
        kv_free(path->stroke_geoms[i].indices);
    }

    /* TODO: save/restore */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

static void add_fill_line(struct path *p, float xc, float yc, float x0, float y0, float x1, float y1)
{
    float v0x = x0 - xc;
    float v0y = y0 - yc;
    float v1x = x1 - xc;
    float v1y = y1 - yc;

    struct geometry *g = NULL;
    int index;

    if (v0x * v1y - v0y * v1x < 0)
    {
        g = &p->fill_geoms[1];

        index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, xc);
        kv_push_back(g->vertices, yc);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);
    }
    else
    {
        g = &p->fill_geoms[0];

        index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, xc);
        kv_push_back(g->vertices, yc);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);
    }

    kv_push_back(g->indices, index);
    kv_push_back(g->indices, index + 1);
    kv_push_back(g->indices, index + 2);
}

static void add_fill_quad(struct path *p, float xc, float yc, float x0, float y0, float x1, float y1, float x2, float y2)
{
    float v0x, v0y, v1x, v1y;

    v0x = x0 - xc;
    v0y = y0 - yc;
    v1x = x2 - xc;
    v1y = y2 - yc;

    if (v0x * v1y - v0y * v1x < 0)
    {
        struct geometry *g = &p->fill_geoms[1];

        int index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, xc);
        kv_push_back(g->vertices, yc);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x2);
        kv_push_back(g->vertices, y2);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->indices, index);
        kv_push_back(g->indices, index + 1);
        kv_push_back(g->indices, index + 2);
    }
    else
    {
        struct geometry *g = &p->fill_geoms[0];

        int index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, xc);
        kv_push_back(g->vertices, yc);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x2);
        kv_push_back(g->vertices, y2);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->indices, index);
        kv_push_back(g->indices, index + 1);
        kv_push_back(g->indices, index + 2);
    }

    v0x = x1 - x0;
    v0y = y1 - y0;
    v1x = x2 - x0;
    v1y = y2 - y0;

    if (v0x * v1y - v0y * v1x < 0)
    {
        struct geometry *g = &p->fill_geoms[3];

        int index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x2);
        kv_push_back(g->vertices, y2);
        kv_push_back(g->vertices, 1);
        kv_push_back(g->vertices, 1);

        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, 0.5f);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->indices, index);
        kv_push_back(g->indices, index + 1);
        kv_push_back(g->indices, index + 2);
    }
    else
    {
        struct geometry *g = &p->fill_geoms[2];

        int index = kv_size(g->vertices) / 4;

        kv_push_back(g->vertices, x2);
        kv_push_back(g->vertices, y2);
        kv_push_back(g->vertices, 1);
        kv_push_back(g->vertices, 1);

        kv_push_back(g->vertices, x0);
        kv_push_back(g->vertices, y0);
        kv_push_back(g->vertices, 0);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->vertices, x1);
        kv_push_back(g->vertices, y1);
        kv_push_back(g->vertices, 0.5f);
        kv_push_back(g->vertices, 0);

        kv_push_back(g->indices, index);
        kv_push_back(g->indices, index + 1);
        kv_push_back(g->indices, index + 2);
    }
}

static void create_fill_geometry(struct path *path)
{
#define c0 coords[icoord]
#define c1 coords[icoord + 1]
#define c2 coords[icoord + 2]
#define c3 coords[icoord + 3]

#define set(x1, y1, x2, y2) ncpx = x1; ncpy = y1; npepx = x2; npepy = y2;

    reduced_path_vec *reduced_paths = &path->reduced_paths;

    size_t i, j;

    for (i = 0; i < 4; ++i)
    {
        kv_init(path->fill_geoms[i].vertices);
        kv_init(path->fill_geoms[i].indices);
    }

    for (i = 0; i < kv_size(*reduced_paths); ++i)
    {
        struct reduced_path *p = &kv_a(*reduced_paths, i);

        size_t num_commands = kv_size(p->commands);
        unsigned char *commands = kv_data(p->commands);
        float *coords = kv_data(p->coords);

        int closed = 0;

        int icoord = 0;

        float spx = 0, spy = 0;
        float cpx = 0, cpy = 0;
        float pepx = 0, pepy = 0;
        float ncpx = 0, ncpy = 0;
        float npepx = 0, npepy = 0;

        float xc, yc;

        for (j = 0; j < num_commands; ++j)
        {
            switch (commands[j])
            {
            case GL_MOVE_TO_AC:
                set(c0, c1, c0, c1);
                spx = ncpx;
                spy = ncpy;
                xc = spx;
                yc = spy;
                icoord += 2;
                break;
            case GL_LINE_TO_AC:
                add_fill_line(path, xc, yc, cpx, cpy, c0, c1);
                set(c0, c1, c0, c1);
                icoord += 2;
                break;
            case GL_QUADRATIC_CURVE_TO_AC:
                add_fill_quad(path, xc, yc, cpx, cpy, c0, c1, c2, c3);
                set(c2, c3, c0, c1);
                icoord += 4;
                break;
            case GL_CLOSE_PATH_AC:
                add_fill_line(path, xc, yc, cpx, cpy, spx, spy);
                set(spx, spy, spx, spy);
                closed = 1;
                break;
            }

            cpx = ncpx;
            cpy = ncpy;
            pepx = npepx;
            pepy = npepy;
        }

        if (closed == 0)
        {
            // TODO: close the fill geometry
        }
    }

#undef c0
#undef c1
#undef c2
#undef c3

#undef set

    path->fill_bounds[0] = 1e30f;
    path->fill_bounds[1] = 1e30f;
    path->fill_bounds[2] = -1e30f;
    path->fill_bounds[3] = -1e30f;

    update_bounds(path->fill_bounds, kv_size(path->fill_geoms[0].vertices), kv_data(path->fill_geoms[0].vertices), 4);
    update_bounds(path->fill_bounds, kv_size(path->fill_geoms[1].vertices), kv_data(path->fill_geoms[1].vertices), 4);
    update_bounds(path->fill_bounds, kv_size(path->fill_geoms[2].vertices), kv_data(path->fill_geoms[2].vertices), 4);
    update_bounds(path->fill_bounds, kv_size(path->fill_geoms[3].vertices), kv_data(path->fill_geoms[3].vertices), 4);

    struct merger4f m;
    init_merger4f(&m);

    path->fill_starts[0] = kv_size(m.indices);
    merge4f(&m, kv_data(path->fill_geoms[0].vertices), kv_data(path->fill_geoms[0].indices), kv_size(path->fill_geoms[0].indices));
    merge4f(&m, kv_data(path->fill_geoms[2].vertices), kv_data(path->fill_geoms[2].indices), kv_size(path->fill_geoms[2].indices));
    path->fill_counts[0] = kv_size(m.indices) - path->fill_starts[0];
    path->fill_starts[1] = kv_size(m.indices);
    merge4f(&m, kv_data(path->fill_geoms[1].vertices), kv_data(path->fill_geoms[1].indices), kv_size(path->fill_geoms[1].indices));
    merge4f(&m, kv_data(path->fill_geoms[3].vertices), kv_data(path->fill_geoms[3].indices), kv_size(path->fill_geoms[3].indices));
    path->fill_counts[1] = kv_size(m.indices) - path->fill_starts[1];

    glBindBuffer(GL_ARRAY_BUFFER, path->fill_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, kv_size(m.vertices) * 4 * sizeof(float), kv_data(m.vertices), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, path->fill_index_buffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, kv_size(m.indices) * sizeof(unsigned short), kv_data(m.indices), GL_STATIC_DRAW);

    free_merger4f(&m);

    for (i = 0; i < 4; ++i)
    {
#if 0
        glBindBuffer(GL_ARRAY_BUFFER, path->fill_geoms[i].vertex_buffer);
        glBufferData(GL_ARRAY_BUFFER, kv_size(path->fill_geoms[i].vertices) * sizeof(float), kv_data(path->fill_geoms[i].vertices), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, path->fill_geoms[i].index_buffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, kv_size(path->fill_geoms[i].indices) * sizeof(unsigned short), kv_data(path->fill_geoms[i].indices), GL_STATIC_DRAW);

        path->fill_geoms[i].count = kv_size(path->fill_geoms[i].indices);
#endif

        kv_free(path->fill_geoms[i].vertices);
        kv_free(path->fill_geoms[i].indices);
    }

    /* TODO: save/restore */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void init_path_rendering()
{
    int i;

    paths = kh_init(path);

    init_segments();

    for (i = 0; i < 256; ++i)
        command_coords[i] = -1;

    command_coords[GL_CLOSE_PATH_AC] = 0;
    command_coords['Z'] = 0;
    command_coords['z'] = 0;
    command_coords[GL_MOVE_TO_AC] = 2;
    command_coords['M'] = 2;
    command_coords[GL_RELATIVE_MOVE_TO_AC] = 2;
    command_coords['m'] = 2;
    command_coords[GL_LINE_TO_AC] = 2;
    command_coords['L'] = 2;
    command_coords[GL_RELATIVE_LINE_TO_AC] = 2;
    command_coords['l'] = 2;
    command_coords[GL_HORIZONTAL_LINE_TO_AC] = 1;
    command_coords['H'] = 1;
    command_coords[GL_RELATIVE_HORIZONTAL_LINE_TO_AC] = 1;
    command_coords['h'] = 1;
    command_coords[GL_VERTICAL_LINE_TO_AC] = 1;
    command_coords['V'] = 1;
    command_coords[GL_RELATIVE_VERTICAL_LINE_TO_AC] = 1;
    command_coords['v'] = 1;
    command_coords[GL_QUADRATIC_CURVE_TO_AC] = 4;
    command_coords['Q'] = 4;
    command_coords[GL_RELATIVE_QUADRATIC_CURVE_TO_AC] = 4;
    command_coords['q'] = 4;
    command_coords[GL_CUBIC_CURVE_TO_AC] = 6;
    command_coords['C'] = 6;
    command_coords[GL_RELATIVE_CUBIC_CURVE_TO_AC] = 6;
    command_coords['c'] = 6;
    command_coords[GL_SMOOTH_QUADRATIC_CURVE_TO_AC] = 2;
    command_coords['T'] = 2;
    command_coords[GL_RELATIVE_SMOOTH_QUADRATIC_CURVE_TO_AC] = 2;
    command_coords['t'] = 2;
    command_coords[GL_SMOOTH_CUBIC_CURVE_TO_AC] = 4;
    command_coords['S'] = 4;
    command_coords[GL_RELATIVE_SMOOTH_CUBIC_CURVE_TO_AC] = 4;
    command_coords['s'] = 4;
    command_coords[GL_SMALL_CCW_ARC_TO_AC] = 5;
    command_coords[GL_RELATIVE_SMALL_CCW_ARC_TO_AC] = 5;
    command_coords[GL_SMALL_CW_ARC_TO_AC] = 5;
    command_coords[GL_RELATIVE_SMALL_CW_ARC_TO_AC] = 5;
    command_coords[GL_LARGE_CCW_ARC_TO_AC] = 5;
    command_coords[GL_RELATIVE_LARGE_CCW_ARC_TO_AC] = 5;
    command_coords[GL_LARGE_CW_ARC_TO_AC] = 5;
    command_coords[GL_RELATIVE_LARGE_CW_ARC_TO_AC] = 5;
    command_coords[GL_RESTART_PATH_AC] = 0;
    command_coords[GL_DUP_FIRST_CUBIC_CURVE_TO_AC] = 4;
    command_coords[GL_DUP_LAST_CUBIC_CURVE_TO_AC] = 4;
    command_coords[GL_RECT_AC] = 4;
    command_coords[GL_CIRCULAR_CCW_ARC_TO_AC] = 5;
    command_coords[GL_CIRCULAR_CW_ARC_TO_AC] = 5;
    command_coords[GL_CIRCULAR_TANGENT_ARC_TO_AC] = 5;
    command_coords[GL_ARC_TO_AC] = 7;
    command_coords['A'] = 7;
    command_coords[GL_RELATIVE_ARC_TO_AC] = 7;
    command_coords['a'] = 7;

    {
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vs_code0, NULL);
        glCompileShader(vs);

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fs_code0, NULL);
        glCompileShader(fs);

        program0 = glCreateProgram();
        glAttachShader(program0, vs);
        glAttachShader(program0, fs);
        glBindAttribLocation(program0, DATA0_POS, "data0");
        glDeleteShader(vs);
        glDeleteShader(fs);
        glLinkProgram(program0);

        matrix0 = glGetUniformLocation(program0, "matrix");
#ifdef OPENGL_ES
        mvp0 = glGetUniformLocation(program0, "mvp");
#endif
    }

    {
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vs_code1, NULL);
        glCompileShader(vs);

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fs_code1, NULL);
        glCompileShader(fs);

        program1 = glCreateProgram();
        glAttachShader(program1, vs);
        glAttachShader(program1, fs);
        glBindAttribLocation(program1, DATA0_POS, "data0");
        glDeleteShader(vs);
        glDeleteShader(fs);
        glLinkProgram(program1);

        matrix1 = glGetUniformLocation(program1, "matrix");
#ifdef OPENGL_ES
        mvp1 = glGetUniformLocation(program0, "mvp");
#endif
    }

    {
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vs_code2, NULL);
        glCompileShader(vs);

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fs_code2, NULL);
        glCompileShader(fs);

        program2 = glCreateProgram();
        glAttachShader(program2, vs);
        glAttachShader(program2, fs);
        glBindAttribLocation(program2, DATA0_POS, "data0");
        glBindAttribLocation(program2, DATA1_POS, "data1");
        glBindAttribLocation(program2, DATA2_POS, "data2");
        glDeleteShader(vs);
        glDeleteShader(fs);
        glLinkProgram(program2);

        matrix2 = glGetUniformLocation(program2, "matrix");
#ifdef OPENGL_ES
        mvp2 = glGetUniformLocation(program0, "mvp");
#endif
    }

    {
        GLuint vs = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vs, 1, &vs_code3, NULL);
        glCompileShader(vs);

        GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fs, 1, &fs_code3, NULL);
        glCompileShader(fs);

        program3 = glCreateProgram();
        glAttachShader(program3, vs);
        glAttachShader(program3, fs);
        glBindAttribLocation(program3, DATA0_POS, "data0");
        glDeleteShader(vs);
        glDeleteShader(fs);
        glLinkProgram(program3);

        matrix3 = glGetUniformLocation(program3, "matrix");
#ifdef OPENGL_ES
        mvp3 = glGetUniformLocation(program0, "mvp");
#endif
    }

#ifdef OPENGL_ES
    memcpy(g_mvp, identity_matrix, 16 * sizeof(float));
#endif
}

void cleanup_path_rendering()
{
    cleanup_segments();
    kh_destroy(path, paths);
}

GLuint glGenPathsAC(GLsizei range)
{
    return gen_paths(range);
}

void glDeletePathsAC(GLuint path, GLsizei range)
{
    delete_paths(path, range);
}

GLboolean glIsPathAC(GLuint path)
{
    return kh_get(path, paths, path) != kh_end(paths);
}

void glPathCommandsAC(GLuint path, GLsizei numCommands, const GLubyte *commands, GLsizei numCoords, GLenum coordType, const GLvoid *coords)
{
    float *coords2;
    if (coordType != GL_BYTE &&
        coordType != GL_UNSIGNED_BYTE &&
        coordType != GL_SHORT &&
        coordType != GL_UNSIGNED_SHORT &&
        coordType != GL_FLOAT)
    {
        // TODO: set error
        return;
    }

    coords2 = (float*)malloc(numCoords * sizeof(float));

    if (coordType == GL_BYTE)
    {
        GLbyte *coords3 = (GLbyte*)coords;
        int i;
        for (i = 0; i < numCoords; ++i)
            coords2[i] = coords3[i];
    }
    else if (coordType == GL_UNSIGNED_BYTE)
    {
        GLubyte *coords3 = (GLubyte*)coords;
        int i;
        for (i = 0; i < numCoords; ++i)
            coords2[i] = coords3[i];
    }
    else if (coordType == GL_SHORT)
    {
        GLshort *coords3 = (GLshort*)coords;
        int i;
        for (i = 0; i < numCoords; ++i)
            coords2[i] = coords3[i];
    }
    else if (coordType == GL_UNSIGNED_SHORT)
    {
        GLushort *coords3 = (GLushort*)coords;
        int i;
        for (i = 0; i < numCoords; ++i)
            coords2[i] = coords3[i];
    }
    else if (coordType == GL_FLOAT)
    {
        GLfloat *coords3 = (GLfloat*)coords;
        int i;
        for (i = 0; i < numCoords; ++i)
            coords2[i] = coords3[i];
    }

    path_commands(path, numCommands, commands, numCoords, coords2);

    free(coords2);
}

static void stencil_stroke_path(GLuint path, GLint reference, GLuint mask, const float matrix[16])
{
    khiter_t iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
        return;

    struct path *p = kh_val(paths, iter);

    if (p->is_stroke_dirty)
    {
        create_stroke_geometry(p);
        p->is_stroke_dirty = 0;
    }

    // save
    GLboolean cmask[4], dmask;
    GLint smask;
    glGetBooleanv(GL_COLOR_WRITEMASK, cmask);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &dmask);
    glGetIntegerv(GL_STENCIL_WRITEMASK, &smask);

    GLint sfunc, sref, svmask;
    glGetIntegerv(GL_STENCIL_FUNC, &sfunc);
    glGetIntegerv(GL_STENCIL_REF, &sref);
    glGetIntegerv(GL_STENCIL_VALUE_MASK, &svmask);

    GLenum fail, zfail, zpass;
    glGetIntegerv(GL_STENCIL_FAIL, &fail);
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_FAIL, &zfail);
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_PASS, &zpass);

    GLint currentProgram;
    glGetIntegerv(GL_CURRENT_PROGRAM, &currentProgram);

    GLint data0Enabled, data1Enabled, data2Enabled;
    glGetVertexAttribiv(DATA0_POS, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &data0Enabled);
    glGetVertexAttribiv(DATA1_POS, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &data1Enabled);
    glGetVertexAttribiv(DATA2_POS, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &data2Enabled);

    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);
    glStencilMask(mask);
    glStencilFunc(GL_ALWAYS, reference, ~0);    // TODO: this should be configured with glPathStencilFuncAC
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

    if (p->stroke_geoms[0].count > 0)
    {
        glUseProgram(program0);
        glUniformMatrix4fv(matrix0, 1, GL_FALSE, matrix);
#ifdef OPENGL_ES
        glUniformMatrix4fv(mvp0, 1, GL_FALSE, g_mvp);
#endif

        glEnableVertexAttribArray(DATA0_POS);

        glBindBuffer(GL_ARRAY_BUFFER, p->stroke_geoms[0].vertex_buffer);
        glVertexAttribPointer(DATA0_POS, 2, GL_FLOAT, GL_FALSE, 8, BUFFER_OFFSET(0));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p->stroke_geoms[0].index_buffer);
        glDrawElements(GL_TRIANGLES, p->stroke_geoms[0].count, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));
    }

    if (p->stroke_geoms[1].count > 0)
    {
        glUseProgram(program2);
        glUniformMatrix4fv(matrix2, 1, GL_FALSE, matrix);
#ifdef OPENGL_ES
        glUniformMatrix4fv(mvp2, 1, GL_FALSE, g_mvp);
#endif

        glEnableVertexAttribArray(DATA0_POS);
        glEnableVertexAttribArray(DATA1_POS);
        glEnableVertexAttribArray(DATA2_POS);

        glBindBuffer(GL_ARRAY_BUFFER, p->stroke_geoms[1].vertex_buffer);
        glVertexAttribPointer(DATA0_POS, 4, GL_FLOAT, GL_FALSE, 48, BUFFER_OFFSET(0 * 4));
        glVertexAttribPointer(DATA1_POS, 4, GL_FLOAT, GL_FALSE, 48, BUFFER_OFFSET(4 * 4));
        glVertexAttribPointer(DATA2_POS, 4, GL_FLOAT, GL_FALSE, 48, BUFFER_OFFSET(8 * 4));
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p->stroke_geoms[1].index_buffer);
        glDrawElements(GL_TRIANGLES, p->stroke_geoms[1].count, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));
    }

    // restore
    if (!data0Enabled)
        glDisableVertexAttribArray(DATA0_POS);
    if (!data1Enabled)
        glDisableVertexAttribArray(DATA1_POS);
    if (!data2Enabled)
        glDisableVertexAttribArray(DATA2_POS);

    glUseProgram(currentProgram);

    glColorMask(cmask[0], cmask[1], cmask[2], cmask[3]);
    glDepthMask(dmask);
    glStencilMask(smask);
    glStencilFunc(sfunc, sref, svmask);
    glStencilOp(fail, zfail, zpass);

    /* TODO: save/restore */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void glStencilStrokePathAC(GLuint path, GLint reference, GLuint mask)
{
    stencil_stroke_path(path, reference, mask, identity_matrix);
}

static void stencil_fill_path(GLuint path, GLenum fill_mode, GLuint mask, const GLfloat matrix[16])
{
    struct path *p = NULL;

    khiter_t iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        // TODO: check if we should set error here
        return;
    }

    GLenum front, back;
    switch (fill_mode)
    {
    case GL_COUNT_UP_AC:
        front = GL_INCR_WRAP;
        back = GL_DECR_WRAP;
        break;
    case GL_COUNT_DOWN_AC:
        front = GL_DECR_WRAP;
        back = GL_INCR_WRAP;
        break;
    case GL_INVERT:
        front = GL_INVERT;
        back = GL_INVERT;
        break;
    default:
        // TODO: check if we should set error here
        return;
    }

    p = kh_val(paths, iter);

    if (p->is_fill_dirty)
    {
        create_fill_geometry(p);
        p->is_fill_dirty = 0;
    }

    // save
    GLboolean cmask[4], dmask;
    GLint smask;
    glGetBooleanv(GL_COLOR_WRITEMASK, cmask);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &dmask);
    glGetIntegerv(GL_STENCIL_WRITEMASK, &smask);

    GLint sfunc, sref, svmask;
    glGetIntegerv(GL_STENCIL_FUNC, &sfunc);
    glGetIntegerv(GL_STENCIL_REF, &sref);
    glGetIntegerv(GL_STENCIL_VALUE_MASK, &svmask);

    GLenum fail, zfail, zpass;
    glGetIntegerv(GL_STENCIL_FAIL, &fail);
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_FAIL, &zfail);
    glGetIntegerv(GL_STENCIL_PASS_DEPTH_PASS, &zpass);

    GLint currentProgram;
    glGetIntegerv(GL_CURRENT_PROGRAM, &currentProgram);

    GLint data0Enabled;
    glGetVertexAttribiv(DATA0_POS, GL_VERTEX_ATTRIB_ARRAY_ENABLED, &data0Enabled);

    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);
    glStencilMask(mask);
    glStencilFunc(GL_ALWAYS, 0, ~0);    // TODO: this should be configured with glPathStencilFuncAC

    glEnableVertexAttribArray(DATA0_POS);

    glBindBuffer(GL_ARRAY_BUFFER, p->fill_vertex_buffer);
    glVertexAttribPointer(DATA0_POS, 4, GL_FLOAT, GL_FALSE, 16, BUFFER_OFFSET(0));
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p->fill_index_buffer);

    glUseProgram(program1);
    glUniformMatrix4fv(matrix1, 1, GL_FALSE, matrix);
#ifdef OPENGL_ES
    glUniformMatrix4fv(mvp1, 1, GL_FALSE, g_mvp);
#endif

    if (p->fill_counts[0] > 0)
    {
        glStencilOp(GL_KEEP, GL_KEEP, front);
        glDrawElements(GL_TRIANGLES, p->fill_counts[0], GL_UNSIGNED_SHORT, BUFFER_OFFSET(p->fill_starts[0] * 2));
    }

    if (p->fill_counts[1] > 0)
    {
        glStencilOp(GL_KEEP, GL_KEEP, back);
        glDrawElements(GL_TRIANGLES, p->fill_counts[1], GL_UNSIGNED_SHORT, BUFFER_OFFSET(p->fill_starts[1] * 2));
    }

    // restore
    if (!data0Enabled)
        glDisableVertexAttribArray(DATA0_POS);

    glUseProgram(currentProgram);

    glColorMask(cmask[0], cmask[1], cmask[2], cmask[3]);
    glDepthMask(dmask);
    glStencilMask(smask);
    glStencilFunc(sfunc, sref, svmask);
    glStencilOp(fail, zfail, zpass);

    /* TODO: save/restore */
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void glStencilFillPathAC(GLuint path, GLenum fillMode, GLuint mask)
{
    stencil_fill_path(path, fillMode, mask, identity_matrix);
}

void glPathParameterfAC(GLuint path, GLenum pname, GLfloat value)
{
    struct path *p = NULL;

    khiter_t iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        // TODO: check if we should set error here
        return;
    }

    p = kh_val(paths, iter);

    switch (pname)
    {
    case GL_PATH_STROKE_WIDTH_AC:
        p->stroke_width = value;
        p->is_stroke_dirty = 1;
        break;
    case GL_PATH_MITER_LIMIT_AC:
        p->miter_limit = value;
        p->is_stroke_dirty = 1;
        break;
    default:
        // TODO: check if we should set error here
        break;
    }


}

void glPathParameteriAC(GLuint path, GLenum pname, GLint value)
{
    khiter_t iter;
    struct path *p;

    iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        // TODO: check if we should set error here
        return;
    }

    p = kh_val(paths, iter);

    switch (pname)
    {
    case GL_PATH_JOIN_STYLE_AC:
        if (value != GL_MITER_REVERT_AC && value != GL_MITER_TRUNCATE_AC && value != GL_BEVEL_AC && value != GL_ROUND_AC && value != GL_NONE)
        {
            /* TODO: check if we should set error here */
            return;
        }
        p->join_style = value;
        p->is_stroke_dirty = 1;
        break;
    case GL_PATH_INITIAL_END_CAP_AC:
        if (value != GL_FLAT && value != GL_SQUARE_AC && value != GL_ROUND_AC && value != GL_TRIANGULAR_AC)
        {
            /* TODO: check if we should set error here */
            return;
        }
        p->initial_end_cap = value;
        p->is_stroke_dirty = 1;
        break;
        break;
    case GL_PATH_TERMINAL_END_CAP_AC:
        if (value != GL_FLAT && value != GL_SQUARE_AC && value != GL_ROUND_AC && value != GL_TRIANGULAR_AC)
        {
            /* TODO: check if we should set error here */
            return;
        }
        p->terminal_end_cap = value;
        p->is_stroke_dirty = 1;
        break;
        break;
    default:
        /* TODO: check if we should set error here */
        break;
    }
}


static int get_path_name(int pathNameType, const void **paths, unsigned int pathBase, unsigned int *pathName)
{
    typedef signed char byte;
    typedef unsigned char ubyte;
    typedef unsigned short ushort;
    typedef unsigned int uint;

    switch (pathNameType)
    {
    case GL_BYTE:
    {
        const byte *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_UNSIGNED_BYTE:
    {
        const ubyte *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_SHORT:
    {
        const short *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_UNSIGNED_SHORT:
    {
        const ushort *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_INT:
    {
        const int *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_UNSIGNED_INT:
    {
        const uint *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_FLOAT:
    {
        const float *p = *paths;
        *pathName = pathBase + p[0];
        *paths = p + 1;
        return 1;
    }
    case GL_2_BYTES:
    {
        const ubyte *p = *paths;
        *pathName = pathBase + (p[0] << 8 | p[1]);
        *paths = p + 2;
        return 1;
    }
    case GL_3_BYTES:
    {
        const ubyte *p = *paths;
        *pathName = pathBase + (p[0] << 16 | p[1] << 8 | p[0]);
        *paths = p + 3;
        return 1;
    }
    case GL_4_BYTES:
    {
        const ubyte *p = *paths;
        *pathName = pathBase + (p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]);
        *paths = p + 4;
        return 1;
    }
    case GL_UTF8_AC:
    {
        const ubyte *p = *paths;
        ubyte c0 = p[0];
        if ((c0 & 0x80) == 0x00)
        {
            /* Zero continuation (0 to 127) */
            *pathName = pathBase + c0;
            p += 1;
        }
        else
        {
            ubyte c1 = p[1];
            if ((c1 & 0xC0) != 0x80)
            {
                /* Stop processing the UTF byte sequence early. */
                return 0;
            }
            if ((c0 & 0xE0) == 0xC0)
            {
                /* One contination (128 to 2047) */
                *pathName = pathBase + ((c1 & 0x3F) | (c0 & 0x1F) << 6);
                if (*pathName < 128)
                {
                    return 0;
                }
                p += 2;
            }
            else
            {
                ubyte c2 = p[2];
                if ((c2 & 0xC0) != 0x80)
                {
                    /* Stop processing the UTF byte sequence early. */
                    return 0;
                }
                if ((c0 & 0xF0) == 0xE0)
                {
                    /* Two continuation (2048 to 55295 and 57344 to 65535) */
                    *pathName = pathBase + ((c2 & 0x3F) | (c1 & 0x3F) << 6 | (c0 & 0xF) << 12);
                    if ((*pathName >= 55296) && (*pathName <= 57343))
                    {
                        /* Stop processing the UTF byte sequence early. */
                        return 0;
                    }
                    if (*pathName < 2048)
                    {
                        return 0;
                    }
                    p += 3;
                }
                else
                {
                    ubyte c3 = p[3];
                    if ((c3 & 0xC0) != 0x80)
                    {
                        /* Stop processing the UTF byte sequence early. */
                        return 0;
                    }
                    if ((c0 & 0xF8) == 0xF0)
                    {
                        /* Three continuation (65536 to 1114111) */
                        *pathName = pathBase + ((c3 & 0x3F) | (c2 & 0x3F) << 6 | (c1 & 0x3F) << 12 | (c0 & 0x7) << 18);
                        if (*pathName < 65536 && *pathName > 1114111)
                        {
                            return 0;
                        }
                        p += 4;
                    }
                    else
                    {
                        /* Skip invalid or restricted encodings. */
                        /* Stop processing the UTF byte sequence early. */
                        return 0;
                    }
                }
            }
        }
        *paths = p;
        return 1;
    }
    case GL_UTF16_AC:
    {
        const ushort *p = *paths;

        ushort s0 = p[0];
        if ((s0 < 0xDB00) || (s0 > 0xDFFF))
        {
            *pathName = pathBase + s0;
            p += 1;
        }
        else
        {
            if ((s0 >= 0xDB00) && (s0 <= 0xDBFF))
            {
                ushort s1 = p[1];
                if ((s1 >= 0xDC00) && (s1 <= 0xDFFF))
                {
                    *pathName = pathBase + (((s0 & 0x3FF) << 10 | (s1 & 0x3FF)) + 0x10000);
                    p += 2;
                }
                else
                {
                    /* Stop processing the UTF byte sequence early. */
                    return 0;
                }
            }
            else
            {
                return 0;
            }
        }
        *paths = p;
        return 1;
    }
    default:  /* TODO: generate INVALID_ENUM */
        return 0;
    }
}

static const float *apply_transform_type(int transform_type, const float *v, float m[16])
{
    switch (transform_type)
    {
    case GL_NONE:
        m[0] = 1; m[4] = 0; m[8] = 0;  m[12] = 0;
        m[1] = 0; m[5] = 1; m[9] = 0;  m[13] = 0;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
        break;
    case GL_TRANSLATE_X_AC:
        m[0] = 1; m[4] = 0; m[8] = 0;  m[12] = v[0];
        m[1] = 0; m[5] = 1; m[9] = 0;  m[13] = 0;
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
        v += 1;
        break;
    case GL_TRANSLATE_Y_AC:
        m[0] = 1; m[4] = 0; m[8] = 0;  m[12] = 0;
        m[1] = 0; m[5] = 1; m[9] = 0;  m[13] = v[0];
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
        v += 1;
        break;
    case GL_TRANSLATE_2D_AC:
        m[0] = 1; m[4] = 0; m[8] = 0;  m[12] = v[0];
        m[1] = 0; m[5] = 1; m[9] = 0;  m[13] = v[1];
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = 0;
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
        v += 2;
        break;
    case GL_TRANSLATE_3D_AC:
        m[0] = 1; m[4] = 0; m[8] = 0;  m[12] = v[0];
        m[1] = 0; m[5] = 1; m[9] = 0;  m[13] = v[1];
        m[2] = 0; m[6] = 0; m[10] = 1; m[14] = v[2];
        m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
        v += 3;
        break;
    case GL_AFFINE_2D_AC:
        m[0] = v[0]; m[4] = v[2]; m[8] = 0;  m[12] = v[4];
        m[1] = v[1]; m[5] = v[3]; m[9] = 0;  m[13] = v[5];
        m[2] = 0;    m[6] = 0;    m[10] = 1; m[14] = 0;
        m[3] = 0;    m[7] = 0;    m[11] = 0; m[15] = 1;
        v += 6;
        break;
    case GL_TRANSPOSE_AFFINE_2D_AC:
        m[0] = v[0]; m[4] = v[1]; m[8] = 0;  m[12] = v[2];
        m[1] = v[3]; m[5] = v[4]; m[9] = 0;  m[13] = v[5];
        m[2] = 0;    m[6] = 0;    m[10] = 1; m[14] = 0;
        m[3] = 0;    m[7] = 0;    m[11] = 0; m[15] = 1;
        v += 6;
        break;
    case GL_AFFINE_3D_AC:
        m[0] = v[0]; m[4] = v[3]; m[8] = v[6];  m[12] = v[9];
        m[1] = v[1]; m[5] = v[4]; m[9] = v[7];  m[13] = v[10];
        m[2] = v[2]; m[6] = v[5]; m[10] = v[8]; m[14] = v[11];
        m[3] = 0;    m[7] = 0;    m[11] = 1;    m[15] = 0;
        v += 12;
        break;
    case GL_TRANSPOSE_AFFINE_3D_AC:
        m[0] = v[0]; m[4] = v[1]; m[8] = v[2];   m[12] = v[3];
        m[1] = v[4]; m[5] = v[5]; m[9] = v[6];   m[13] = v[7];
        m[2] = v[8]; m[6] = v[9]; m[10] = v[10]; m[14] = v[11];
        m[3] = 0;    m[7] = 0;    m[11] = 1;     m[15] = 0;
        v += 12;
        break;
    default:  /* TODO: generate INVALID_ENUM */
        break;
    }
    return v;
}

void glStencilFillPathInstancedAC(GLsizei numPaths, GLenum pathNameType, const void* paths, GLuint pathBase, GLenum fillMode, GLuint mask, GLenum transformType, const GLfloat *transformValues)
{
    int i;
    const float *v;
    
    v = transformValues;
    for (i = 0; i < numPaths; ++i)
    {
        float m[16];
        unsigned int pathName;

        v = apply_transform_type(transformType, v, m);
        if (v == NULL)
            return;
        
        if (!get_path_name(pathNameType, &paths, pathBase, &pathName))
            return;

        if (glIsPathAC(pathName))
            stencil_fill_path(pathName, fillMode, mask, m);
    }
}

void glStencilStrokePathInstancedAC(GLsizei numPaths, GLenum pathNameType, const void* paths, GLuint pathBase, GLint reference, GLuint mask, GLenum transformType, const GLfloat *transformValues)
{
    int i;
    const float *v;

    v = transformValues;
    for (i = 0; i < numPaths; ++i)
    {
        float m[16];
        unsigned int pathName;

        v = apply_transform_type(transformType, v, m);
        if (v == NULL)
            return;

        if (!get_path_name(pathNameType, &paths, pathBase, &pathName))
            return;

        if (glIsPathAC(pathName))
            stencil_stroke_path(pathName, reference, mask, m);
    }
}

void glGetPathBoundingBoxInstancedAC(GLenum boundingBoxType, GLsizei numPaths, GLenum pathNameType, const void* paths, GLuint pathBase, GLenum transformType, const GLfloat *transformValues, GLfloat *result)
{
    int i;
    int hasBounds = 0;
    float boundsUnion[4], bounds[4];

    const float *v = transformValues;
    for (i = 0; i < numPaths; i++)
    {
        unsigned int pathName;
        if (!get_path_name(pathNameType, &paths, pathBase, &pathName))
            return;

        if (glIsPathAC(pathName))
        {
            glGetPathParameterfvAC(pathName, boundingBoxType, bounds);
            switch (transformType)
            {
            case GL_NONE:
                break;
            case GL_TRANSLATE_X_AC:
                bounds[0] += v[0];
                bounds[2] += v[0];
                v += 1;
                break;
            case GL_TRANSLATE_Y_AC:
                bounds[1] += v[0];
                bounds[3] += v[0];
                v += 1;
                break;
            case GL_TRANSLATE_2D_AC:
                bounds[0] += v[0];
                bounds[1] += v[1];
                bounds[2] += v[0];
                bounds[3] += v[1];
                v += 2;
                break;
            case GL_TRANSLATE_3D_AC: /* ignores v[2] */
                bounds[0] += v[0];
                bounds[1] += v[1];
                bounds[2] += v[0];
                bounds[3] += v[1];
                v += 3;
                break;
            case GL_AFFINE_2D_AC:
                bounds[0] = bounds[0] * v[0] + bounds[0] * v[2] + v[4];
                bounds[1] = bounds[1] * v[1] + bounds[1] * v[3] + v[5];
                bounds[2] = bounds[2] * v[0] + bounds[2] * v[2] + v[4];
                bounds[3] = bounds[3] * v[1] + bounds[3] * v[3] + v[5];
                v += 6;
                break;
            case GL_TRANSPOSE_AFFINE_2D_AC:
                bounds[0] = bounds[0] * v[0] + bounds[0] * v[1] + v[2];
                bounds[1] = bounds[1] * v[3] + bounds[1] * v[4] + v[5];
                bounds[2] = bounds[2] * v[0] + bounds[2] * v[1] + v[2];
                bounds[3] = bounds[3] * v[3] + bounds[3] * v[4] + v[5];
                v += 6;
                break;
            case GL_AFFINE_3D_AC:  /* ignores v[2], v[5], v[6..8], v[11] */
                bounds[0] = bounds[0] * v[0] + bounds[0] * v[3] + v[9];
                bounds[1] = bounds[1] * v[1] + bounds[1] * v[4] + v[10];
                bounds[2] = bounds[2] * v[0] + bounds[2] * v[3] + v[9];
                bounds[3] = bounds[3] * v[1] + bounds[3] * v[4] + v[10];
                v += 12;
                break;
            case GL_TRANSPOSE_AFFINE_3D_AC:  /* ignores v[2], v[6], v[8..11] */
                bounds[0] = bounds[0] * v[0] + bounds[0] * v[1] + v[3];
                bounds[1] = bounds[1] * v[4] + bounds[1] * v[5] + v[7];
                bounds[2] = bounds[2] * v[0] + bounds[2] * v[1] + v[3];
                bounds[3] = bounds[3] * v[4] + bounds[3] * v[5] + v[7];
                v += 12;
                break;
            default:  /* TODO: generate INVALID_ENUM */
                break;
            }

            if (bounds[0] > bounds[2])
            {
                float t = bounds[2];
                bounds[2] = bounds[0];
                bounds[0] = t;
            }

            if (bounds[1] > bounds[3])
            {
                float t = bounds[3];
                bounds[3] = bounds[1];
                bounds[1] = t;
            }

            if (hasBounds)
            {
                boundsUnion[0] = MIN(boundsUnion[0], bounds[0]);
                boundsUnion[1] = MIN(boundsUnion[1], bounds[1]);
                boundsUnion[2] = MAX(boundsUnion[2], bounds[2]);
                boundsUnion[3] = MAX(boundsUnion[3], bounds[3]);
            }
            else
            {
                memcpy(boundsUnion, bounds, 4 * sizeof(float));
                hasBounds = 1;
            }
        }
    }

    if (hasBounds)
        memcpy(result, boundsUnion, 4 * sizeof(float));
}


void glGetPathParameterfvAC(GLuint path, GLenum param, GLfloat *value)
{
    khiter_t iter;
    struct path *p;

    iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        /* TODO: check if we should set error here */
        return;
    }

    p = kh_val(paths, iter);

    switch (param)
    {
    case GL_PATH_OBJECT_BOUNDING_BOX_AC:
        break;
    case GL_PATH_FILL_BOUNDING_BOX_AC:
        if (p->is_fill_dirty)
        {
            create_fill_geometry(p);
            p->is_fill_dirty = 0;
        }
        value[0] = p->fill_bounds[0];
        value[1] = p->fill_bounds[1];
        value[2] = p->fill_bounds[2];
        value[3] = p->fill_bounds[3];
        break;
    case GL_PATH_STROKE_BOUNDING_BOX_AC:
        if (p->is_stroke_dirty)
        {
            create_stroke_geometry(p);
            p->is_stroke_dirty = 0;
        }
        value[0] = p->stroke_bounds[0];
        value[1] = p->stroke_bounds[1];
        value[2] = p->stroke_bounds[2];
        value[3] = p->stroke_bounds[3];
        break;
    default:
        /* TODO: check if we should set error here */
        break;
    }

}

void glGetPathParameterivAC(GLuint path, GLenum param, GLint *value)
{


}


void glPathDashArrayAC(GLuint path, GLsizei dashCount, const GLfloat *dashArray)
{
    GLsizei i;

    khiter_t iter;
    struct path *p;

    iter = kh_get(path, paths, path);
    if (iter == kh_end(paths))
    {
        /* TODO: generate error INVALID_OPERATION */
        return;
    }

    p = kh_val(paths, iter);

    if (dashCount < 0)
    {
        /* TODO: generate error INVALID_VALUE */
        return;
    }

    for (i = 0; i < dashCount; ++i)
    {
        if (dashArray[i] < 0)
        {
            /* TODO: generate error INVALID_VALUE */
            return;
        }
    }

    if (dashCount == 0)
    {
        p->num_dashes = 0;
        free(p->dashes);
        p->dashes = NULL;

        p->dash_length = 0;

        p->is_stroke_dirty = 1;
    }
    else
    {
        p->num_dashes = dashCount;
        free(p->dashes);
        p->dashes = malloc(sizeof(float) * dashCount);
        memcpy(p->dashes, dashArray, sizeof(float) * dashCount);

        p->dash_length = 0;
        for (i = 0; i < dashCount; ++i)
            p->dash_length += dashArray[i];

        p->is_stroke_dirty = 1;
    }
}

#ifdef OPENGL_ES
void glLoadPathMatrix(const GLfloat *m)
{
    memcpy(g_mvp, m, 16 * sizeof(GLfloat));
}
void glGetPathMatrix(GLfloat *m)
{
    memcpy(m, g_mvp, 16 * sizeof(GLfloat));
}
#endif

#if 0

static float ccw(const float *p1, const float *p2, const float *p3)
{
    return (p2[0] - p1[0])*(p3[1] - p1[1]) - (p2[1] - p1[1])*(p3[0] - p1[0]);
}

static int compr(const void *ptr1, const void *ptr2)
{
    const float *p1 = (const float*)ptr1;
    const float *p2 = (const float*)ptr2;

    if (p1[0] < p2[0])
        return -1;
    if (p1[0] > p2[0])
        return 1;
    if (p1[1] < p2[1])
        return -1;
    if (p1[1] > p2[1])
        return 1;

    return 0;
}

void evaluate(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float t, float *px, float *py)
{
    *px = Ax * t * t + Bx * t + Cx;
    *py = Ay * t * t + By * t + Cy;
}

void tangent(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float t, float *px, float *py)
{
    float tx = 2 * Ax * t + Bx;
    float ty = 2 * Ay * t + By;

    float l = sqrt(tx * tx + ty * ty);

    tx /= l;
    ty /= l;

    *px = tx;
    *py = ty;
}

void offset(float Ax, float Ay, float Bx, float By, float Cx, float Cy, float t, float d, float *px, float *py)
{
    float tx, ty;
    tangent(Ax, Ay, Bx, By, Cx, Cy, t, &tx, &ty);

    float x, y;
    evaluate(Ax, Ay, Bx, By, Cx, Cy, t, &x, &y);

    *px = x - d * ty;
    *py = y + d * tx;
}

float intersect(float x0, float y0, float tx0, float ty0,
    float x1, float y1, float tx1, float ty1)
{
    /* Solve[x0 + tx0 t0 == x1 + tx1 t1 && y0 + ty0 t0 == y1 + ty1 t1, {t0, t1}] */
    return -((-(ty1*x0) + ty1*x1 + tx1*y0 - tx1*y1) / (tx1*ty0 - tx0*ty1));
}

int convex_hull_quad(float x0, float y0, float x1, float y1, float x2, float y2, float d, float hull[8 * 2 * 2])
{
    int index = 0;
    float vertices[8 * 2];

    float Ax = x0 - 2 * x1 + x2;
    float Ay = y0 - 2 * y1 + y2;
    float Bx = 2 * (x1 - x0);
    float By = 2 * (y1 - y0);
    float Cx = x0;
    float Cy = y0;

    if ((x2 - x0) * (y1 - y0) - (y2 - y0) * (x1 - x0) < 0)
        d = -d;

    /* outer curve */
    float px0, py0, tx0, ty0, ox0, oy0;
    float px1, py1, tx1, ty1, ox1, oy1;

    evaluate(Ax, Ay, Bx, By, Cx, Cy, 0, &px0, &py0);
    tangent(Ax, Ay, Bx, By, Cx, Cy, 0, &tx0, &ty0);
    offset(Ax, Ay, Bx, By, Cx, Cy, 0, d, &ox0, &oy0);

    evaluate(Ax, Ay, Bx, By, Cx, Cy, 1, &px1, &py1);
    tangent(Ax, Ay, Bx, By, Cx, Cy, 1, &tx1, &ty1);
    offset(Ax, Ay, Bx, By, Cx, Cy, 1, d, &ox1, &oy1);

    float m = tan((atan2(ty0, tx0) + atan2(ty1, tx1)) / 2);
    float t = (-By + Bx * m) / (2 * (Ay - Ax * m));                 /* Solve[(2  Ay t + By) / (2 Ax t + Bx) == m, {t}] */

    float pxm, pym, txm, tym, oxm, oym;
    evaluate(Ax, Ay, Bx, By, Cx, Cy, t, &pxm, &pym);
    tangent(Ax, Ay, Bx, By, Cx, Cy, t, &txm, &tym);
    offset(Ax, Ay, Bx, By, Cx, Cy, t, d, &oxm, &oym);

    vertices[index++] = ox0;
    vertices[index++] = oy0;

    vertices[index++] = ox1;
    vertices[index++] = oy1;

    float t0 = intersect(ox0, oy0, tx0, ty0, oxm, oym, txm, tym);
    vertices[index++] = ox0 + tx0 * t0;
    vertices[index++] = oy0 + ty0 * t0;

    float t1 = intersect(ox1, oy1, tx1, ty1, oxm, oym, txm, tym);
    vertices[index++] = ox1 + tx1 * t1;
    vertices[index++] = oy1 + ty1 * t1;

    /* inner curve */
    /*
    px = Ax t^2 + Bx t + Cx
    py = Ay t^2 + By t + Cy
    nx = -D[py, t]
    ny = D[px, t]
    nnx = nx / (nx ^ 2 + ny ^ 2) ^ (1/2)
    nny = ny / (nx ^ 2 + ny ^ 2) ^ (1/2)
    ox = Simplify[px + d * nnx]
    oy = Simplify[py + d * nny]

    Solve[D[ox, t] == 0, {t}]
    */
#define Power pow
#define Sqrt sqrt

    t0 = (4 * Ax*Bx + 4 * Ay*By - Sqrt(Power(-4 * Ax*Bx - 4 * Ay*By, 2) - 4 * (-4 * Power(Ax, 2) - 4 * Power(Ay, 2))*(-Power(Bx, 2) - Power(By, 2) + Power(2, 0.6666666666666666)*Power(Power(Ay*Bx - Ax*By, 2)*Power(d, 2), 0.3333333333333333)))) / (2.*(-4 * Power(Ax, 2) - 4 * Power(Ay, 2)));
    t1 = (4 * Ax*Bx + 4 * Ay*By + Sqrt(Power(-4 * Ax*Bx - 4 * Ay*By, 2) - 4 * (-4 * Power(Ax, 2) - 4 * Power(Ay, 2))*(-Power(Bx, 2) - Power(By, 2) + Power(2, 0.6666666666666666)*Power(Power(Ay*Bx - Ax*By, 2)*Power(d, 2), 0.3333333333333333)))) / (2.*(-4 * Power(Ax, 2) - 4 * Power(Ay, 2)));

    if (0 <= t0 && t0 <= 1)
    {
        float x, y;
        offset(Ax, Ay, Bx, By, Cx, Cy, t0, -d, &x, &y);
        vertices[index++] = x;
        vertices[index++] = y;
    }
    if (0 <= t1 && t1 <= 1)
    {
        float x, y;
        offset(Ax, Ay, Bx, By, Cx, Cy, t1, -d, &x, &y);
        vertices[index++] = x;
        vertices[index++] = y;
    }

#undef Power
#undef Sqrt

    offset(Ax, Ay, Bx, By, Cx, Cy, 0, -d, &ox0, &oy0);
    offset(Ax, Ay, Bx, By, Cx, Cy, 1, -d, &ox1, &oy1);
    vertices[index++] = ox0;
    vertices[index++] = oy0;
    vertices[index++] = ox1;
    vertices[index++] = oy1;

    int npoints = index / 2;

    int num_indices;
    int indices[16];
    {
        qsort(vertices, npoints, sizeof(float)* 2, compr);
        int i, t, k = 0;

        /* lower hull */
        for (i = 0; i < npoints; ++i)
        {
            while (k >= 2 && ccw(&vertices[indices[k - 2] * 2], &vertices[indices[k - 1] * 2], &vertices[i * 2]) <= 0) --k;
            indices[k++] = i;
        }

        /* upper hull */
        for (i = npoints - 2, t = k + 1; i >= 0; --i) {
            while (k >= t && ccw(&vertices[indices[k - 2] * 2], &vertices[indices[k - 1] * 2], &vertices[i * 2]) <= 0) --k;
            indices[k++] = i;
        }

        num_indices = k - 1;
    }

    for (int i = 0; i < num_indices; ++i)
    {
        int index = indices[i];
        hull[i * 2] = vertices[index * 2];
        hull[i * 2 + 1] = vertices[index * 2 + 1];
    }

    return num_indices;
}

/* axis of symmerty */
/*
x = Ax t^2 + Bx t + Cx
y = Ay t^2 + By t + Cy
xp = D[x, t]
xpp = D[xp, t]
yp = D[y, t]
ypp = D[yp, t]
k = ((xp * ypp) - (yp * xpp)) / ((xp^2 + yp^2)^(3/2))
Solve[D[k, t] == 0, {t}]
*/


static const char *fs_code =
"#define M_PI 3.1415926535897932384626433832795                       \n"
"                                                                            \n"
"uniform vec2 A, B, C;                      \n"
"uniform float strokeWidth;                \n"
"uniform float t0, t1;                \n"
"                                                                            \n"
"varying float b, c, d;                    \n"
"varying vec2 pos;          \n"
"                                                                            \n"
"vec2 evaluateQuadratic(float t)                                            \n"
"{                                                                            \n"
"   return (A * t + B) * t + C;                                            \n"
"}                                                                            \n"
"                                                                            \n"
"float f(float t)                                            \n"
"{                                                                            \n"
"   return ((t + b) * t + c) * t + d;                                            \n"
"}                                                                            \n"
"                                                                            \n"
"bool check(float t)                                                        \n"
"{                                                                            \n"
"   if (-1e-3 <= t && t <= 1 + 1e-3)                                                \n"
"   {                                                                        \n"
"      vec2 q = evaluateQuadratic(t) - pos;                                    \n"
"      if (dot(q, q) <= strokeWidth)                                        \n"
"         return false;                                                        \n"
"   }                                                                        \n"
"                                                                            \n"
"   return true;                                                            \n"
"}                                                                            \n"
"                                                                       \n"
"float estimate(vec4 v)                                                       \n"
"{                                                                      \n"
"  float xm = (v.x + v.y) / 2;                                          \n"
"  float ym = f(xm);                                                    \n"
"  float d = (v.y - v.x) / 2;                                           \n"
"  float a = (v.z + v.w - 2 * ym) / (2 * d * d);                        \n"
"  float b = (v.w - v.z) / (2 * d);                                     \n"
"  return xm - (2 * ym) / (b * (1 + sqrt(1 - (4 * a * ym) / (b * b)))); \n"
"}                                                                      \n"
"                                                                       \n"
"vec4 update(vec4 v, float x)                                           \n"
"{                                                                      \n"
"  vec2 m = vec2(x, f(x));                                              \n"
"  vec2 c;                                                              \n"
"  c.x = (sign(v.z * m.y) + 1) / 2;                                     \n"
"  c.y = 1 - c.x;                                                       \n"
"  return c.yxyx * v + c.xyxy * m.xxyy;                                 \n"
"}                                                                      \n"
"                                                                       \n"
"float cbrt(float x)                                                        \n"
"{                                                                            \n"
"   return sign(x) * pow(abs(x), 1.0 / 3.0);                                \n"
"}                                                                            \n"
"// http://stackoverflow.com/questions/11854907/calculate-the-length-of-a-segment-of-a-quadratic-bezier                                                                       \n"
"void dash(float t)                                                                     \n"
"{                                                                       \n"
"                                                                       \n"
"}                                                                       \n"
"                                                                       \n"
"                                                                       \n"
"                                                                       \n"
"                                                                       \n"
"                                                                       \n"
"                                                                       \n"
"void main(void)                                                        \n"
"{                                                                      \n"
#if 1
"  vec4 v = vec4(t0, t1, f(t0), f(t1));                                 \n"
"  v = update(v, estimate(v));                                          \n"
"  v = update(v, estimate(v));                                          \n"
"  v = update(v, estimate(v));                                          \n"
"  float t0 = estimate(v);                                              \n"
"                                                                       \n"
"  float e = (t0 + b) / 2;                                              \n"
"  float f = 2 * e * t0 + c;                                            \n"
"  float disc = e * e - f;                                              \n"
"  if (disc < 0)                                                        \n"
"  {                                                                    \n"
"    if (check(t0)) discard;                                            \n"
"  }                                                                    \n"
"  else                                                                 \n"
"  {                                                                    \n"
"    disc = sqrt(disc);                                                 \n"
"    float t1 = -e - disc;                                              \n"
"    float t2 = -e + disc;                                              \n"
"    if (check(t0) && check(t1) && check(t2)) discard;                  \n"
"  }                                                                    \n"
#else
"  float p = (3 * c - b * b) / 3;                                                  \n"
"  float q = (2 * b * b * b - 9 * b * c + 27 * d) / 27;                            \n"
"  float offset = -b / 3;                                                  \n"
"  float disc = q * q / 4 + p * p * p / 27;                                \n"
"                                                                            \n"
"  if (disc >= 0.0)                                                            \n"
"  {                                                                        \n"
"    float c1 = -q / 2.0;                                                    \n"
"    float c2 = sqrt(disc);                                                    \n"
"    float t0 = cbrt(c1 + c2) + cbrt(c1 - c2) + offset;                                                                      \n"
"    if (check(t0)) discard;                                            \n"
"  }                                                                        \n"
"  else                                                                    \n"
"  {                                                                        \n"
"    float cos_3_theta = 3.0 * q * sqrt(-3.0 / p) / (2.0 * p);            \n"
"    float theta = acos(cos_3_theta) / 3.0;                                \n"
"    float r = 2.0 * sqrt(-p / 3.0);                                        \n"
"    float t0 = r * cos(theta) + offset;                                                                        \n"
"    float t1 = r * cos(theta + 2.0 * M_PI / 3.0) + offset;                                                                        \n"
"    float t2 = r * cos(theta + 4.0 * M_PI / 3.0) + offset;                                                                        \n"
"    if (check(t0) && check(t1) && check(t2)) discard;                  \n"
"   }                                                                        \n"
#endif
"  gl_FragColor = vec4(1, 1, 1, 1);                                         \n"
"}                                                                      \n";


static void clip_poly(const float *poly, int npoly,
    float px, float py, float nx, float ny,
    float *poly0, int *npoly0,
    float *poly1, int *npoly1)
{
    int index0 = 0;
    int index1 = 0;

    for (int i = 0; i < npoly; ++i)
    {
        int i1 = (i + 1) % npoly;

        double x0 = poly[i * 2];
        double y0 = poly[i * 2 + 1];
        double x1 = poly[i1 * 2];
        double y1 = poly[i1 * 2 + 1];

        bool b0 = ((x0 - px) * ny - (y0 - py) * nx) >= 0;
        bool b1 = ((x1 - px) * ny - (y1 - py) * nx) >= 0;

        float xp, yp;
        if (b0 != b1)
        {
            float t = intersect(x0, y0, x1 - x0, y1 - y0, px, py, nx, ny);
            xp = (1 - t) * x0 + t * x1;
            yp = (1 - t) * y0 + t * y1;
        }

        if (!b0 && b1)
        {
            // out (xp, yp, x1, y1)
            // out (xp, yp)
            poly0[index0++] = xp;
            poly0[index0++] = yp;
            poly0[index0++] = x1;
            poly0[index0++] = y1;
            poly1[index1++] = xp;
            poly1[index1++] = yp;
        }
        else if (b0 && b1)
        {
            // out (x1, y1)
            // out none
            poly0[index0++] = x1;
            poly0[index0++] = y1;
        }
        else if (b0 && !b1)
        {
            // out (xp, yp)
            // out (xp, yp, x1, y1)
            poly0[index0++] = xp;
            poly0[index0++] = yp;
            poly1[index1++] = xp;
            poly1[index1++] = yp;
            poly1[index1++] = x1;
            poly1[index1++] = y1;
        }
        else if (!b0 && !b1)
        {
            // out none
            // out (x1, y1) 
            poly1[index1++] = x1;
            poly1[index1++] = y1;
        }
    }

    *npoly0 = index0 / 2;
    *npoly1 = index1 / 2;

}


#endif
