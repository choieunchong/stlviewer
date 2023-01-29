#ifndef MYOPENGLWIDGET_H
#define MYOPENGLWIDGET_H

#include <QWidget>
#include <QSlider>

QT_BEGIN_NAMESPACE
namespace Ui { class MyOpenGLWidget; }
QT_END_NAMESPACE

class MyOpenGLWidget : public QWidget
{
    Q_OBJECT

public:
    MyOpenGLWidget(QWidget *parent = nullptr);
    ~MyOpenGLWidget();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

private:
    Ui::MyOpenGLWidget *ui;
     QSlider *m_slider;
};
#endif // MYOPENGLWIDGET_H
