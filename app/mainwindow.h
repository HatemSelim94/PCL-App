#pragma once

#include <QMainWindow>
#include <pcl/visualization/pcl_visualizer.h>

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "core/cloud_session.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/// Thin Qt shell around pclapp::AnyCloudSession. Every button handler follows
/// the same shape: read parameters from the widgets, call one session method,
/// show the result. All point-type dispatch happens once, in withSession().
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    // Named after the widgets in mainwindow.ui so connectSlotsByName() wires them.
    void on_loadFile_clicked();
    void on_showPointCloud_clicked();
    void on_Filter_clicked();
    void on_showFilteredCloud_clicked();
    void on_Cluster_clicked();
    void on_showClusters_clicked();
    void on_saveFilteredCloud_clicked();
    void on_ConvertFile_clicked();

private:
    /// Runs `action(session)` on the loaded session, whatever its point type.
    /// Shows a hint when nothing is loaded and reports any exception thrown.
    template <typename Action>
    void withSession(Action&& action);

    template <typename PointT>
    void showCloud(const pclapp::CloudPtr<PointT>& cloud, const std::string& id);

    template <typename PointT>
    void showClusters(const std::vector<pclapp::CloudPtr<PointT>>& clusters);

    pclapp::VoxelGridParams voxelParamsFromUi() const;
    pclapp::EuclideanClusterParams clusterParamsFromUi() const;
    void reportError(const std::exception& error);
    void refreshView();  ///< Redraws the 3D view after its content changed.

    std::unique_ptr<Ui::MainWindow> ui_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pclapp::AnyCloudSession session_;
};
