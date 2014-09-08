#include "SLVideoWidget.h"


//#ifdef __APPLE__
//    #include <OpenGL/glu.h>
//#else
//    #include <GL/glu.h>
//#endif

void SLVideoWidget::initializeGL(){
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_TEXTURE_2D);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    #ifndef GL_EXT_texture_swizzle
        textureSwizzleProgram = new QGLShaderProgram(this);
        textureSwizzleProgram->addShaderFromSourceCode(QGLShader::Fragment,
            "uniform sampler2D texture;\n"
            "void main(void){\n"
            "   gl_FragColor = texture2D(texture, gl_TexCoord[0].st).rrra;\n"
            "}");
        textureSwizzleProgram->link();
    #endif
}

void SLVideoWidget::setGrayScale(bool enable){
    if(enable){
        // Set texture swizzle to display gray value images on red channel
        #ifdef GL_EXT_texture_swizzle
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_RED);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_RED);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
        #else
            textureSwizzleProgram->bind();
        #endif
    } else {
        #ifdef GL_EXT_texture_swizzle
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_R, GL_RED);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_GREEN);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_BLUE);
        #else
            textureSwizzleProgram->release();
        #endif
    }
}

void SLVideoWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0); // set origin to bottom left corner
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    aspectRatioWidget = (float)w/(float)h;

}

void SLVideoWidget::showFrame(CameraFrame frame){
    this->makeCurrent();
    this->setGrayScale(true);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.width, frame.height, 0, GL_RED, GL_UNSIGNED_BYTE, frame.memory);

    aspectRatioTexture = (float)frame.width/(float)frame.height;

    this->updateGL();
}


void SLVideoWidget::showFrameCV(cv::Mat frame){
    this->makeCurrent();

    if(frame.type() == CV_8U){
        this->setGrayScale(true);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_RED, GL_UNSIGNED_BYTE, frame.ptr());
    } else if(frame.type() == CV_8UC3){
        this->setGrayScale(false);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, frame.ptr());
    } else if(frame.type() == CV_16UC1){
        this->setGrayScale(true);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_RED, GL_UNSIGNED_SHORT, frame.ptr());
    } else if(frame.type() == CV_32FC1){
        this->setGrayScale(true);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.cols, frame.rows, 0, GL_RED, GL_FLOAT, frame.ptr());
    }
    glFinish();
//std::cout << "Showing frame!" << std::endl;

    aspectRatioTexture = (float)frame.cols/(float)frame.rows;

    this->updateGL();
}

void SLVideoWidget::paintGL() {

    // calculate image size
    float width, height;
    if(aspectRatioTexture > aspectRatioWidget) {
        width = this->width();
        height = this->height() * aspectRatioWidget / aspectRatioTexture;
    } else {
        height = this->height();
        width = this->width() / aspectRatioWidget * aspectRatioTexture;
    }

    // center on screen
    float offsetX = (this->width() - width)/2.0;
    float offsetY = (this->height() - height)/2.0;

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(offsetX, offsetY);

        glTexCoord2f(1, 0);
        glVertex2f(offsetX + width, offsetY);

        glTexCoord2f(1, 1);
        glVertex2f(offsetX + width, offsetY + height);

        glTexCoord2f(0, 1);
        glVertex2f(offsetX, offsetY + height);
    glEnd();


}
