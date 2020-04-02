#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    cloud.reset (new PointCloudT);
    selected_cloud.reset (new PointCloudT);

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    // 初始化字段
    pointSize = ui->pointSize_spinBox->value();
    cloudIndex = 0;
    haveAdded = false;

    // 初始化颜色
    color = Qt::red;
    // 初始化界面内容
    ui->selectMode_rbtn->setChecked(true);
    QLabel *label = new QLabel("按下X后拖动鼠标以选择点云，再次按下X生效",this);
    ui->statusBar->addPermanentWidget(label);
    ui->nowCategory_label->setText(QString::number(cloudIndex));

    // 添加初始点云
    PointCloudT::Ptr newcloud(new PointCloudT);
    clouds.push_back(newcloud);
    idList.push_back(QString::number(cloudIndex).toUtf8().constData());
}

MainWindow::~MainWindow()
{
    delete ui;
}

/*
 *
 */
void MainWindow::pp_callback(const pcl::visualization::AreaPickingEvent& event, void* args)
{
    // 获取被框选的点的indices
    std::vector< int > indices;
    if (event.getPointsIndices(indices)==-1)
        return;

    if(indices.size()==0)
    {
        return;
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudColor(clouds[cloudIndex], color.red(), color.green(), color.blue());

    // 选择模式，根据indices从原始点云中选择点，加入新点云
    if(ui->selectMode_rbtn->isChecked())
    {
        for (int i = 0; i < indices.size(); ++i)
        {
            clouds[cloudIndex]->points.push_back(cloud->points.at(indices[i]));
        }

        if(!haveAdded)
        {
            // 未添加过的新类别，生成新类别的id，添加新的点云
            idList.push_back(QString::number(cloudIndex).toUtf8().constData());
            viewer->addPointCloud<PointT>(clouds[cloudIndex], cloudColor, idList[cloudIndex]);
            haveAdded = true;
        }
        else
        {
            // 当前类别已存在，直接更新
            viewer->updatePointCloud<PointT>(clouds[cloudIndex], cloudColor, idList[cloudIndex]);
        }
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, idList[cloudIndex]);
    }
    // 删除模式，根据indices从已选择的点云中删除点
    else if(ui->deleteMode_rbtn->isChecked())
    {
        //PointCloudT::iterator index = clouds[cloudIndex]->begin();
        for (int i = 0; i < indices.size(); ++i)
        {
            clouds[cloudIndex]->erase(clouds[cloudIndex]->begin() + indices[i]);
        }
        viewer->updatePointCloud<PointT>(clouds[cloudIndex], cloudColor, idList[cloudIndex]);
    }
    ui->qvtkWidget->update ();
}

/*
 *
 */
void MainWindow::KeyDownCallback(const pcl::visualization::KeyboardEvent& event, void* args)
{
    if (event.isCtrlPressed())
    {
        viewer->getInteractorStyle()->StartSelect();
    }

}

/*
 *
 */
void MainWindow::on_actionOpen_triggered()
{
    QColor initColor = Qt::green;

    // Show a dialog to select files
    QString qPath = QFileDialog::getOpenFileName(this, tr("Open pcd"), tr("/home/clat/Data"), tr("Point Cloud Files(*.pcd)"));

    if(qPath.isEmpty())
    {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
        return;
    }

    // Convert QString to std::string
    cloudID = qPath.toUtf8().constData();

    // Open the PCD file
    if (pcl::io::loadPCDFile<PointT> (cloudID, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read pcd file\n");
    }

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudColor(cloud, initColor.red(), initColor.green(), initColor.blue());

    viewer->registerAreaPickingCallback (&MainWindow::pp_callback, *this, (void*)&cloud);
    viewer->registerKeyboardCallback(&MainWindow::KeyDownCallback, *this);
    vtkSmartPointer<vtkRenderedAreaPicker> area_picker = vtkSmartPointer<vtkRenderedAreaPicker>::New();
    viewer->getRenderWindow()->GetInteractor()->SetPicker(area_picker);

    viewer->addPointCloud<PointT> (cloud, cloudColor, cloudID);
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

/*
 *
 */
void MainWindow::on_actionSave_triggered()
{
    QString qPath = QFileDialog::getSaveFileName(this, tr("Save pcd"), tr("/home/clat/Data"), tr("Point Cloud Files(*.pcd)"));

    if(qPath.isEmpty())
    {
        QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
        return;
    }

    // Convert QString to std::string
    std::string path = qPath.toUtf8().constData();

    pcl::io::savePCDFileASCII (path, *cloud);
}

/*
 * Select a color.
 */
void MainWindow::on_actionSelect_Color_triggered()
{
    color = QColorDialog::getColor(color, this);
}

void MainWindow::on_pointSize_spinBox_valueChanged(int arg1)
{
    pointSize = arg1;
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudID);
    for(int i=0;i<idList.size();i++)
    {
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, idList[i]);
    }
    ui->qvtkWidget->update ();
}

void MainWindow::on_NewCategory_btn_clicked()
{
    PointCloudT::Ptr newcloud(new PointCloudT);
    clouds.push_back(newcloud);
    cloudIndex++;
    haveAdded = false;
    ui->nowCategory_label->setText(QString::number(cloudIndex));
}

void MainWindow::on_clear_btn_clicked()
{
    clouds[cloudIndex]->clear();
    viewer->removePointCloud (QString::number(cloudIndex).toUtf8().constData());
    ui->qvtkWidget->update ();
    haveAdded = false;
}

void MainWindow::on_actionClearAll_triggered()
{
    viewer->removeAllPointClouds ();
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}
