#include "myopenglwidget.h"
#include "ui_myopenglwidget.h"

MyOpenGLWidget::MyOpenGLWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MyOpenGLWidget)
{
    ui->setupUi(this);
    //connect(m_slider,SIGNAL( m_slider->valueChanged()), this, SLOT(&MyOpenGLWidget::update));
    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 100);
    m_slider->setGeometry(10, 10, 500, 30);
}

MyOpenGLWidget::~MyOpenGLWidget()
{
    delete ui;
}

void MyOpenGLWidget::initializeGL()
{
    // Set up the OpenGL context and state
}

void MyOpenGLWidget::paintGL()
{
    // Render the scene using the current slider value
    int value = m_slider->value();
    //...
}

void MyOpenGLWidget::resizeGL(int w, int h)
{
    // Handle window resizing
}


