/*
 *  Author: Benjamin Langmann (b.langmann@gmx.de)
 *  Date: 2016
 */

#ifndef PROJECTORQTGL_H
#define PROJECTORQTGL_H

#include <vector>
#include <deque>
#include <string>
#include <sys/types.h>

#include <GL/glew.h>

#include <QGLWidget>
#include <QMutex>
#include <QWaitCondition>
#include <QThread>

#include "Projector.h"

struct Texture
{
    size_t ID;
    const unsigned char *tex;
    unsigned int texWidth;
    unsigned int texHeight;
    bool ownsData;

    void takeOwnership()
    {
        if (ownsData) return;
        unsigned char *newBuffer = new unsigned char[3*texWidth*texHeight];
        memcpy(newBuffer, tex, 3*texWidth*texHeight);
        tex = newBuffer;
        ownsData = true;
    }

    Texture() : ownsData(false) {}

    ~Texture()
    {
        if (ownsData)
            delete[] tex;
    }
};

class ProjectorQtGLWidget : public QGLWidget
{
    Q_OBJECT

public:
    // Interface function
    ProjectorQtGLWidget(QWidget * parent = 0);

    QWaitCondition m_waitCond;

    void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight)
    {
        Texture *newTex = new Texture;
        newTex->ID = patternNumber;
        newTex->tex = tex;
        newTex->texWidth = texWidth;
        newTex->texHeight = texHeight;
        newTex->takeOwnership();
        m_texturesToUpload.push_back(newTex);
    }

    void displayPattern(unsigned int patternNumber)
    {
        m_display = patternNumber;
        synchronizedUpdate();
    }

    void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height)
    {
        m_texture.tex = tex;
        m_texture.texWidth = width;
        m_texture.texHeight = height;
        m_display = -3;
        synchronizedUpdate();
    }

    void displayBlack() {m_display = -1; synchronizedUpdate();}
    void displayWhite() {m_display = -2; synchronizedUpdate();}

protected:
    int m_display;
    Texture m_texture;
    std::deque<Texture*> m_texturesToUpload;
    std::vector<GLuint> frameBuffers;

    void initializeGL();

    void resizeGL(int w, int h);

    void paintGL();

    void mouseDoubleClickEvent(QMouseEvent *)
    {
        if (isFullScreen())
            showNormal();
        else
            showFullScreen();
    }

    void synchronizedUpdate()
    {
        if (QThread::currentThread() != this->thread())
        {
            update();
            QMutex mutex;
            mutex.lock();
            m_waitCond.wait(&mutex);
            mutex.unlock();
        }
        else
            updateGL();
    }
};

class HelperObject : public QObject
{
    Q_OBJECT
public:
    HelperObject() : QObject(), m_glWidget(NULL) {}

    ~HelperObject() {
        if (m_glWidget)
            m_glWidget->deleteLater();
    }

    ProjectorQtGLWidget* m_glWidget;

public slots:
    void show() {
        m_glWidget = new ProjectorQtGLWidget();
        m_glWidget->showFullScreen();
    }
};

// ProjectorOpenGL implementations
class ProjectorQtGL : public Projector
{
    public:
        // Interface function
        ProjectorQtGL();

        // Define preset pattern sequence and upload to GPU
        void setPattern(unsigned int patternNumber, const unsigned char *tex, unsigned int texWidth, unsigned int texHeight);
        void displayPattern(unsigned int patternNumber);
        // Upload and display pattern on the fly
        void displayTexture(const unsigned char *tex, unsigned int width, unsigned int height);
        void displayBlack();
        void displayWhite();
        void getScreenRes(unsigned int *nx, unsigned int *ny);
        ~ProjectorQtGL();
private:
        HelperObject* m_helper;
};

#endif
