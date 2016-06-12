/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#include "ProjectorQtGL.h"

#include <QDesktopWidget>
#include <QApplication>
#include <stdio.h>

ProjectorQtGLWidget::ProjectorQtGLWidget(QWidget * parent) : QGLWidget(parent), m_display(-2)
{
    m_texture.tex = NULL;

    // Set up fragment shader
    GLenum err = glewInit();
    if(err != GLEW_OK)
        std::cerr << "ProjectorOpenGL: Could not initialize GLEW!" << std::endl;

    QDesktopWidget deskTopwidget;
    int currentScreen = deskTopwidget.screenNumber(this);
    int countScreens = deskTopwidget.screenCount();
    int newScreen = 0;
    for (int i = countScreens-1; i >= 0; --i)
    {
        if (i != currentScreen)
        {
            newScreen = i;
            break;
        }
    }
    QRect geometry = deskTopwidget.screenGeometry(newScreen);
    setGeometry(geometry.x(), geometry.y(), geometry.width(), geometry.height());
    deskTopwidget.deleteLater();
}

void ProjectorQtGLWidget::initializeGL()
{
    glEnable(GL_TEXTURE_2D);

    glViewport(0, 0, (GLint) width(), (GLint) height());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1, 1, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void ProjectorQtGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, (GLint)w, (GLint)h);
}

void ProjectorQtGLWidget::paintGL()
{
    while (!m_texturesToUpload.empty())
    {
        Texture *tex = m_texturesToUpload.front();
        m_texturesToUpload.pop_front();

        if (tex->ID+1 == frameBuffers.size()+1)
        {
            GLuint frameBuffer;

            // Generate frame buffer object
            glGenFramebuffers(1, &frameBuffer);
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, frameBuffer);
            frameBuffers.push_back(frameBuffer);

            // Generate render buffer object to store pixel data
            GLuint renderBuffer;
            glGenRenderbuffers(1,&renderBuffer);
            glBindRenderbuffer(GL_RENDERBUFFER, renderBuffer);
            glRenderbufferStorage(GL_RENDERBUFFER,GL_RGBA8, width(), height());

            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderBuffer);

            GLint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            if (status != GL_FRAMEBUFFER_COMPLETE)
              std::cout << "Error Framebuffer Complete" << std::endl;
            else
              std::cout << "Framebuffer Complete" << std::endl;

        } else if (tex->ID < frameBuffers.size()) {

            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, frameBuffers[tex->ID]);

        } else if (tex->ID+1 > frameBuffers.size()+1){

            std::cerr << "ProjectorOpenGL: cannot set pattern " << tex->ID << " before setting " << frameBuffers.size() << " -- " << tex->ID-1 << std::endl;
            return;
        }

        // Render pattern into buffer
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex->texWidth, tex->texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, tex->tex);

        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        float texWidthf = (float)width()/tex->texWidth;
        float texHeightf = (float)height()/tex->texHeight;

        glBegin(GL_QUADS);
            glTexCoord2f(0, 0); glVertex2i(0, 0);
            glTexCoord2f(texWidthf, 0); glVertex2i(1, 0);
            glTexCoord2f(texWidthf, texHeightf); glVertex2i(1, 1);
            glTexCoord2f(0, texHeightf); glVertex2i(0, 1);
        glEnd();

        delete tex;

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    if (m_display == -1)
    {
        glClearColor(0.0, 0.0, 0.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    else if (m_display == -2)
    {
        glClearColor(1.0, 1.0, 1.0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
    }
    else if (m_display == -3)
    {
        if (m_texture.tex == NULL)
        {
            return;
        }
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_texture.texWidth, m_texture.texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, m_texture.tex);

        //    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

        float texWidthf = (float)width()/m_texture.texWidth;
        float texHeightf = (float)height()/m_texture.texHeight;

        glBegin(GL_QUADS);
         glTexCoord2f(0, 0); glVertex2i(0, 0);
         glTexCoord2f(texWidthf, 0); glVertex2i(1, 0);
         glTexCoord2f(texWidthf, texHeightf); glVertex2i(1, 1);
         glTexCoord2f(0, texHeightf); glVertex2i(0, 1);
        glEnd();
    }
    else if(m_display >= 0 && m_display < frameBuffers.size())
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, frameBuffers[m_display]);

        glBlitFramebuffer(0, 0, width(), height(), 0, 0, width(), height(), GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    else
    {
        std::cerr << "ProjectorOpenGL: cannot display pattern " << m_display<< "! Out of bounds." << std::endl;
        return;
    }

    std::cout << "Displaying pattern: " << m_display << std::endl;
    m_waitCond.wakeAll();
}

ProjectorQtGL::ProjectorQtGL()
{
    m_helper = new HelperObject();
    if (QThread::currentThread() != QApplication::instance()->thread())
    {
        m_helper->moveToThread(QApplication::instance()->thread());
        QMetaObject::invokeMethod(m_helper, "show");
        while (!m_helper->m_glWidget) {

        }
    }
    else {
        m_helper->show();
    }
}

ProjectorQtGL::~ProjectorQtGL()
{
    m_helper->deleteLater();
}

void ProjectorQtGL::setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight)
{
    if (m_helper->m_glWidget)
        m_helper->m_glWidget->setPattern(patternNumber, tex, texWidth, texHeight);
}

void ProjectorQtGL::displayPattern(unsigned int patternNumber)
{
    if (m_helper->m_glWidget)
        m_helper->m_glWidget->displayPattern(patternNumber);
}

void ProjectorQtGL::displayTexture(const unsigned char *tex, unsigned int texWidth, unsigned int texHeight)
{
    if (m_helper->m_glWidget)
        m_helper->m_glWidget->displayTexture(tex, texWidth, texHeight);
}

void ProjectorQtGL::displayBlack()
{
    if (m_helper->m_glWidget)
        m_helper->m_glWidget->displayBlack();
}

void ProjectorQtGL::displayWhite()
{
    if (m_helper->m_glWidget)
        m_helper->m_glWidget->displayWhite();
}

void ProjectorQtGL::getScreenRes(unsigned int *nx, unsigned int *ny)
{
    if (m_helper->m_glWidget)
    {
        *nx = m_helper->m_glWidget->width();
        *ny = m_helper->m_glWidget->height();
    }
    else
    {
        *nx = 100;
        *ny = 100;
    }
}
