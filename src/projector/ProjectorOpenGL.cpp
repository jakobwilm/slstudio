#include "ProjectorOpenGL.h"

#include "Projector.h"
#include "OpenGLContext.h"

ProjectorOpenGL::ProjectorOpenGL(unsigned int _screenNum){

    // Create the OpenGL context
    context = new OpenGLContext(_screenNum);

    // OpenGL setup
    context->makeContextCurrent();

    glEnable(GL_TEXTURE_2D);

    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    // Set up for 1:1 pixel mapping in the z=0 plane. Upper left corner is (0,0).
    glViewport(0, 0, context->getScreenResX(), context->getScreenResY());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 1, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Set up fragment shader
    GLenum err = glewInit();
    if(err != GLEW_OK)
        std::cerr << "ProjectorOpenGL: Could not initialize GLEW!" << std::endl;

//    const GLchar *vertexShaderSource =
//    "void main(){\n"
//    "    gl_Position = ftransform();\n"
//    "}";


//    const GLchar *fragmentShaderSource =
//    "uniform sampler2D texture;\n"
//    "void main(void){\n"
//    "   gl_FragColor = texture2D(texture, gl_TexCoord[0].st);\n"
//    "}";

//    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
//    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
//    glCompileShader(vertexShader);
//    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
//    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
//    glCompileShader(fragmentShader);

//    GLint status;
//    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &status);
//    if(status != GL_TRUE)
//        std::cerr << "ProjectorOpenGL: Could not compile vertex shader!" << std::endl;

//    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &status);
//    if(status != GL_TRUE)
//        std::cerr << "ProjectorOpenGL: Could not compile fragment shader!" << std::endl;

//    int len = 0;
//    glGetShaderiv(vertexShader, GL_INFO_LOG_LENGTH, &len);
//    char* log = new char[len];
//    glGetShaderInfoLog(vertexShader, len, NULL, log);
//    std::cout << log << std::endl;

//    glGetShaderiv(fragmentShader, GL_INFO_LOG_LENGTH, &len);
//    glGetShaderInfoLog(fragmentShader, len, NULL, log);
//    std::cout << log << std::endl;

//    shaderProgram = glCreateProgram();
//    glAttachShader(shaderProgram, vertexShader);
//    glAttachShader(shaderProgram, fragmentShader);
//    glLinkProgram(shaderProgram);
//    glUseProgram(shaderProgram);

    context->flush();
}

void ProjectorOpenGL::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

    context->makeContextCurrent();

    if(patternNumber+1 == frameBuffers.size()+1){

        GLuint frameBuffer;

        // Generate frame buffer object
        glGenFramebuffers(1, &frameBuffer);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, frameBuffer);
        frameBuffers.push_back(frameBuffer);

        // Generate render buffer object to store pixel data
        GLuint renderBuffer;
        glGenRenderbuffers(1,&renderBuffer);
        glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER,GL_RGBA8, context->getScreenResX(), context->getScreenResY());

        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderBuffer);

        GLint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE)
          std::cout << "Error Framebuffer Complete" << std::endl;
        else
          std::cout << "Framebuffer Complete" << std::endl;

    } else if (patternNumber < frameBuffers.size()) {

        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, frameBuffers[patternNumber]);

    } else if (patternNumber+1 > frameBuffers.size()+1){

        std::cerr << "ProjectorOpenGL: cannot set pattern " << patternNumber << " before setting " << frameBuffers.size() << " -- " << patternNumber-1 << std::endl;
        return;

    }

    // Render pattern into buffer
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);

    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    float texWidthf = (float)context->getScreenResX()/texWidth;
    float texHeightf = (float)context->getScreenResY()/texHeight;

    glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2i(0, 0);
        glTexCoord2f(texWidthf, 0); glVertex2i(1, 0);
        glTexCoord2f(texWidthf, texHeightf); glVertex2i(1, 1);
        glTexCoord2f(0, texHeightf); glVertex2i(0, 1);
    glEnd();

}

void ProjectorOpenGL::displayPattern(unsigned int patternNumber){

    if(patternNumber+1 > frameBuffers.size()){
        std::cerr << "ProjectorOpenGL: cannot display pattern " << patternNumber << "! Out of bounds." << std::endl;
        return;
    }

    context->makeContextCurrent();

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, frameBuffers[patternNumber]);

    glBlitFramebuffer(0, 0, context->getScreenResX(), context->getScreenResY(), 0, 0, context->getScreenResX(), context->getScreenResY(), GL_COLOR_BUFFER_BIT, GL_NEAREST);

//    glClear(GL_COLOR_BUFFER_BIT);

    //glBindTexture(GL_TEXTURE_2D, textures[patternNumber].texName);

//    GLint locTex = glGetUniformLocation(shaderProgram, "texture");
//    glUniform1i(locTex, 0);

    context->flush();
}

void ProjectorOpenGL::displayTexture(const unsigned char *tex, unsigned int texWidth, unsigned int texHeight){

    context->makeContextCurrent();

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, tex);
    
//    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    
    float texWidthf = (float)context->getScreenResX()/texWidth;
    float texHeightf = (float)context->getScreenResY()/texHeight;
    
    glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2i(0, 0);
        glTexCoord2f(texWidthf, 0); glVertex2i(1, 0);
        glTexCoord2f(texWidthf, texHeightf); glVertex2i(1, 1);
        glTexCoord2f(0, texHeightf); glVertex2i(0, 1);
    glEnd();
    
    context->flush();
}

void ProjectorOpenGL::displayBlack(){
    context->makeContextCurrent();
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    context->flush();
}

void ProjectorOpenGL::displayWhite(){
    context->makeContextCurrent();
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    context->flush();
}

void ProjectorOpenGL::getScreenRes(unsigned int *nx, unsigned int *ny){
    *nx = context->getScreenResX();
    *ny = context->getScreenResY();
}

ProjectorOpenGL::~ProjectorOpenGL(){
    context->makeContextCurrent();
    delete context;
}

