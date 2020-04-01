#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QColorDialog>
#include <QDebug>
#include <QSurfaceFormat>

#include <iostream>
#include <vector>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderedAreaPicker.h>
#include <QVTKOpenGLWidget.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkPointPicker.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args);
    void KeyDownCallback(const pcl::visualization::KeyboardEvent& event, void* args);
    ~MainWindow();

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer <vtkGenericOpenGLRenderWindow> _renderWindow;
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr selected_cloud;
    std::vector<PointCloudT::Ptr> clouds;

private slots:
    void on_actionOpen_triggered();

    void on_actionSelect_Color_triggered();

    void on_actionSave_triggered();

    void on_pointSize_spinBox_valueChanged(int arg1);

    void on_NewCategory_btn_clicked();

    void on_clear_btn_clicked();

    void on_actionClearAll_triggered();

private:
    Ui::MainWindow *ui;
    QColor color;
    std::string cloudID;
    int pointSize;
    int cloudIndex;
    bool haveAdded;
    std::vector<std::string> idList;

};

#endif // MAINWINDOW_H
