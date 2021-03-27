#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <pclprocessor.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pclprocessor.cpp>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT // macro at the beginning of the class definition is necessary for all
    //classes that define signals or slots
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QString fileName;
    Types::PointType dataType;
    PclProcessor<pcl::PointXYZ>* pclProcessorXYZ;
    PclProcessor<pcl::PointXYZI>* pclProcessorXYZI;
private slots:
    void on_loadFile_clicked();

    void on_showPointCloud_clicked();

    void on_voxelLeafSize_valueChanged(double arg1);

    void on_distanceTolerance_valueChanged(double arg1);

    void on_minClusterPoints_valueChanged(int arg1);

    void on_maxClusterPoints_valueChanged(int arg1);

    void on_Cluster_clicked();

    void on_showClusters_clicked();

    void on_Filter_clicked();

    void on_showFilteredCloud_clicked();

    void on_ConvertFile_clicked();

    void on_saveFilteredCloud_clicked();

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;

private:
    Ui::MainWindow *ui;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudXYZI;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloudXYZ;
    pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloudXYZI;
};



#endif // MAINWINDOW_H
