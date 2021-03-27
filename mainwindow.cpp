#include "mainwindow.h"
#include "./ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle ("PCL Application");

    this->dataType = Types::PointType::Empty;

    this->cloudXYZ.reset (new pcl::PointCloud<pcl::PointXYZ>);
    this->cloudXYZI.reset (new pcl::PointCloud<pcl::PointXYZI>);

    this->processedCloudXYZ.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->processedCloudXYZI.reset(new pcl::PointCloud<pcl::PointXYZI>);

    this->pclProcessorXYZ = new PclProcessor<pcl::PointXYZ>;
    this->pclProcessorXYZI = new PclProcessor<pcl::PointXYZI>;

    this->pclProcessorXYZ->clusterManager = new ClusterManager<pcl::PointXYZ>;
    this->pclProcessorXYZI->clusterManager = new ClusterManager<pcl::PointXYZI>;

    this->pclProcessorXYZ->filterManager = new FilterManager<pcl::PointXYZ>;
    this->pclProcessorXYZI->filterManager = new FilterManager<pcl::PointXYZI>;

    this->pclProcessorXYZ->clusterManager->ecClustering = new EculeadieanClustering<pcl::PointXYZ>;
    this->pclProcessorXYZI->clusterManager->ecClustering = new EculeadieanClustering<pcl::PointXYZI>;

    this->pclProcessorXYZ->filterManager->voxelFilter = new VoxelFilter<pcl::PointXYZ>;
    this->pclProcessorXYZI->filterManager->voxelFilter = new VoxelFilter<pcl::PointXYZI>;

    this->pclProcessorXYZ->fileManager = new FileManager<pcl::PointXYZ>;
    this->pclProcessorXYZI->fileManager = new FileManager<pcl::PointXYZI>;



    viewer.reset (new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    viewer->addPointCloud<pcl::PointXYZ>(this->processedCloudXYZ, "cloudXYZ");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

MainWindow::~MainWindow()
{
    delete ui;
    cloudXYZ.reset ();
    processedCloudXYZ.reset();
    cloudXYZI.reset ();
    processedCloudXYZI.reset();
}


void MainWindow::on_loadFile_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,"Select a .txt/ .pcd file","/home", "Files(*.txt *.pcd)");
    if(fileName !=NULL)
    {
        this->fileName = fileName;
        if(ui->XYZData->isChecked())
        {
            std::for_each(this->pclProcessorXYZ->clusters.begin(), this->pclProcessorXYZ->clusters.end(), [](auto pCloud){pCloud.reset();});
            this->pclProcessorXYZ->clusters.clear();
            this->dataType=Types::PointType::XYZ;
            this->cloudXYZ.reset (new pcl::PointCloud<pcl::PointXYZ>);
            this->processedCloudXYZ.reset (new pcl::PointCloud<pcl::PointXYZ>);
            this->pclProcessorXYZ->fileManager->load(this->cloudXYZ, this->fileName.toStdString());
            QString msg = QString("number of points:%1").arg(this->cloudXYZ->width);
            QMessageBox::information(this, "File loaded successfully!",msg);
            pcl::copyPointCloud(*(this->cloudXYZ), *(this->processedCloudXYZ));

        }
        else if(ui->XYZIData->isChecked())
        {
            std::for_each(this->pclProcessorXYZI->clusters.begin(), this->pclProcessorXYZI->clusters.end(), [](auto pCloud){pCloud.reset();});
            this->pclProcessorXYZI->clusters.clear();
            this->dataType=Types::PointType::XYZI;
            this->cloudXYZI.reset (new pcl::PointCloud<pcl::PointXYZI>);
            this->processedCloudXYZI.reset (new pcl::PointCloud<pcl::PointXYZI>);
            this->pclProcessorXYZI->fileManager->load(this->cloudXYZI, this->fileName.toStdString());
            QString msg = QString("number of points:%1").arg(this->cloudXYZI->width);
            QMessageBox::information(this, "File loaded successfully!",msg);
            pcl::copyPointCloud(*(this->cloudXYZI), *(this->processedCloudXYZI));
        }
    }
}

void MainWindow::on_showPointCloud_clicked()
{
    this->viewer->removeAllPointClouds();
    ui->qvtkWidget->update ();
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->viewer->addPointCloud<pcl::PointXYZ>(this->cloudXYZ, "cloudXYZ");
                break;
            }
        case Types::PointType::XYZI:
            {
                this->viewer->addPointCloud<pcl::PointXYZI>(this->cloudXYZI, "cloudXYZI");
                break;
            }
        default:
         {

           }
    }
    ui->qvtkWidget->update ();
}




void MainWindow::on_voxelLeafSize_valueChanged(double arg1)
{
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->pclProcessorXYZ->filterManager->voxelFilter->set_voxelLeafSize(arg1);
            }
            break;
        case Types::PointType::XYZI:
            {
                this->pclProcessorXYZI->filterManager->voxelFilter->set_voxelLeafSize(arg1);
             }
            break;
        default:
            {

            }
    }


}


void MainWindow::on_Filter_clicked()
{
    QString msg="";
    switch (this->dataType){
        case Types::PointType::XYZ:
                {
                    this->pclProcessorXYZ->filterManager->voxelFilter->vFilter(this->cloudXYZ, this->processedCloudXYZ);
                    msg = QString("number of points before/after filtering: %1 / %2").arg(this->cloudXYZ->width).arg(this->processedCloudXYZ->width);
                    QMessageBox::information(this, "Filtering result",msg);
                }
                break;
        case Types::PointType::XYZI:
                {
                    this->pclProcessorXYZI->filterManager->voxelFilter->vFilter(this->cloudXYZI, this->processedCloudXYZI);
                    msg = QString("number of points before/after filtering: %1 / %2").arg(this->cloudXYZI->width).arg(this->processedCloudXYZI->width);
                    QMessageBox::information(this, "Filtering result",msg);
                }
                break;
        default:
                {
                    QMessageBox::information(this, "Warning!",QString("Please load a file first."));
                }
    }




}


void MainWindow::on_showFilteredCloud_clicked()
{
    this->viewer->removeAllPointClouds();
    ui->qvtkWidget->update ();

    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->viewer->addPointCloud<pcl::PointXYZ>(this->processedCloudXYZ, "processedCloudXYZ");
            }
            break;
        case Types::PointType::XYZI:
            {
                this->viewer->addPointCloud<pcl::PointXYZI>(this->processedCloudXYZI, "processedCloudXYZI");
            }
            break;
        default:
            {
                QMessageBox::information(this, "Warning!",QString("Please load a file first."));
            }

    }
    ui->qvtkWidget->update ();
}


void MainWindow::on_distanceTolerance_valueChanged(double arg1)
{
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->pclProcessorXYZ->clusterManager->ecClustering->set_distanceTolerance(arg1);
            }
            break;
        case Types::PointType::XYZI:
            {
                this->pclProcessorXYZI->clusterManager->ecClustering->set_distanceTolerance(arg1);
            }
            break;
        default:
            {

            }
    }

}


void MainWindow::on_minClusterPoints_valueChanged(int arg1)
{
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->pclProcessorXYZ->clusterManager->ecClustering->set_minClusterSize(arg1);
            }
            break;
        case Types::PointType::XYZI:
            {
                this->pclProcessorXYZI->clusterManager->ecClustering->set_minClusterSize(arg1);
            }
            break;
        default:
        {

        }

    }
}


void MainWindow::on_maxClusterPoints_valueChanged(int arg1)
{
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
                this->pclProcessorXYZ->clusterManager->ecClustering->set_maxClusterSize(arg1);
            }
            break;
        case Types::PointType::XYZI:
            {
                this->pclProcessorXYZI->clusterManager->ecClustering->set_maxClusterSize(arg1);
            }
            break;
        default:
        {

        }

    }

}



void MainWindow::on_Cluster_clicked()
{
    QString msg="";
    switch (this->dataType){
        case Types::PointType::XYZ:
            {
            std::for_each(this->pclProcessorXYZ->clusters.begin(), this->pclProcessorXYZ->clusters.end(), [](auto pCloud){pCloud.reset();});
            this->pclProcessorXYZ->clusters.clear();
            this->pclProcessorXYZ->clusterManager->ecClustering->cluster(this->processedCloudXYZ,this->pclProcessorXYZ->clusters);
            msg = QString("number of clusters found: %1").arg(this->pclProcessorXYZ->clusters.size());
            QMessageBox::information(this, "Clustering result",msg);
            }
            break;
        case Types::PointType::XYZI:
            {
            std::for_each(this->pclProcessorXYZI->clusters.begin(), this->pclProcessorXYZI->clusters.end(), [](auto pCloud){pCloud.reset();});
            this->pclProcessorXYZI->clusters.clear();
            this->pclProcessorXYZI->clusterManager->ecClustering->cluster(this->processedCloudXYZI,this->pclProcessorXYZI->clusters);
            msg = QString("number of clusters found: %1").arg(this->pclProcessorXYZI->clusters.size());
            QMessageBox::information(this, "Clustering result",msg);
            }
            break;
        default:
            {
                QMessageBox::information(this, "Warning!",QString("Please load a file first."));
            }
    }

}


void MainWindow::on_showClusters_clicked()
{
    uint_fast8_t id = 0;
    pcl::GlasbeyLUT colors;
    this->viewer->removeAllPointClouds();
    ui->qvtkWidget->update ();
    switch(this->dataType)
    {
        case Types::PointType::XYZ:
            {
                for(auto cluster:this->pclProcessorXYZ->clusters)
                    {
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cluster, colors.at(id).r, colors.at(id).g, colors.at(id).b);
                        this->viewer->addPointCloud<pcl::PointXYZ>(cluster,rgb , "cloud"+std::to_string(id));

                        ui->qvtkWidget->update ();
                        ++id;
                    }
                    break;
            }
        case Types::PointType::XYZI:
        {
            for(auto cluster:this->pclProcessorXYZI->clusters)
                {
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rgb (cluster, colors.at(id).r, colors.at(id).g, colors.at(id).b);
                    this->viewer->addPointCloud<pcl::PointXYZI>(cluster,rgb , "cloud"+std::to_string(id));

                    ui->qvtkWidget->update ();
                    ++id;
                }
                break;
        }

    }


}


void MainWindow::on_ConvertFile_clicked()
{
    switch(this->dataType)
    {
        case Types::PointType::XYZ:
            {
        this->pclProcessorXYZ->fileManager->convert(this->cloudXYZ, this->fileName.toStdString());
        QMessageBox::information(this, "File converted successfully",QString("The new file located in the old file directory"));
    }
        break;

        case Types::PointType::XYZI:
    {
        this->pclProcessorXYZI->fileManager->convert(this->cloudXYZI, this->fileName.toStdString());
        QMessageBox::information(this, "File converted successfully",QString("The new file located in the old file directory"));
       }
        break;
    default:
        {
           QMessageBox::information(this, "Warning!",QString("Please load a file first."));
        }
    }

}



void MainWindow::on_saveFilteredCloud_clicked()
{
    switch(this->dataType)
    {
        case Types::PointType::XYZ:
            {
        this->pclProcessorXYZ->fileManager->save(this->processedCloudXYZ, this->fileName.toStdString());
        QMessageBox::information(this, "Filtered data saved successfully!",QString("The new file is in the old file directory"));
    }
        break;

        case Types::PointType::XYZI:
    {
        this->pclProcessorXYZI->fileManager->save(this->processedCloudXYZI, this->fileName.toStdString());
        QMessageBox::information(this, "Filtered data saved successfully!",QString("The new file is in the old file directory"));
       }
        break;
    default:
        {
           QMessageBox::information(this, "Warning!",QString("Please load a file first."));
        }
    }

}
