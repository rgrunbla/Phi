
/* static const char *nodesVertexShaderSource = "#version 330 core\n"
    "attribute highp vec4 posAttr;"
    "attribute lowp vec4 colAttr;"
    "varying lowp vec4 col;"
    "uniform mat4 model;"
    "uniform mat4 view;"
    "uniform mat4 projection;"
    "void main() {"
    "   col = colAttr;"
    "   gl_Position = projection * view * model * posAttr;"
    "   gl_PointSize = 10.0f;"
    "}";
 */


static const char *nodesVertexShaderSource = "#version 330 core\n"
    "attribute highp vec4 posAttr;"
    "uniform mat4 model;"
    "uniform mat4 view;"
    "uniform mat4 projection;"
    "varying lowp vec4 col;"
    "void main() {"
    "   col = vec4(float((gl_VertexID % 6) == 0 || (gl_VertexID % 6) == 1),"
    "              float((gl_VertexID % 6) == 2 || (gl_VertexID % 6) == 3),"
    "              float((gl_VertexID % 6) == 4 || (gl_VertexID % 6) == 5),"
    "              1.0f);"
    "   gl_Position = projection * view * model * posAttr;"
    "}";


static const char *nodesFragmentShaderSource = "#version 330 core\n"
    "varying lowp vec4 col;"
    "void main(void) {"
    "   gl_FragColor = col;\n"
    "}";

const char *gridVertexShaderSource =  "#version 330 core\n"
    "attribute highp vec4 posAttr;"
    "uniform mat4 model;"
    "uniform mat4 view;"
    "uniform mat4 projection;"
    "void main() {"
    "   gl_Position = projection * view * model * posAttr;"
    "}";

static const char *gridFragmentShaderSource = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "void main(void) {"
    "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
    "}";