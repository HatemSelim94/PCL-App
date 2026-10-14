#pragma once

#include <QMainWindow>
#include <QString>
#include <pcl/visualization/pcl_visualizer.h>

#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "core/cloud_session.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

/// Thin Qt shell around pclapp::AnyCloudSession. Every button handler follows
/// the same shape: read parameters from the widgets, call one session method,
/// show the result. All point-type dispatch happens once, in withSession().
///
/// Buttons are enabled as the workflow allows: nothing before a file is
/// loaded, "Show clusters" only once clusters exist (see updateControls()).
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    // Named after the widgets in mainwindow.ui so connectSlotsByName() wires them.
    void on_actionOpen_triggered();
    void on_actionAbout_triggered();
    void on_loadButton_clicked();
    void on_showSourceButton_clicked();
    void on_downsampleButton_clicked();
    void on_showDownsampledButton_clicked();
    void on_clusterButton_clicked();
    void on_showClustersButton_clicked();
    void on_saveButton_clicked();
    void on_convertButton_clicked();

private:
    /// Runs `action(session)` on the loaded session, whatever its point type,
    /// and reports any exception thrown. Does nothing when no file is loaded.
    template <typename Action>
    void withSession(Action&& action);

    template <typename PointT>
    void showCloud(const pclapp::CloudPtr<PointT>& cloud, const std::string& id);

    template <typename PointT>
    void showClusters(const std::vector<pclapp::CloudPtr<PointT>>& clusters);

    pclapp::VoxelGridParams voxelParamsFromUi() const;
    pclapp::EuclideanClusterParams clusterParamsFromUi() const;
    void reportError(const std::exception& error);
    void refreshView();     ///< Redraws the 3D view after its content changed.
    void updateControls();  ///< Enables the buttons the current state allows.

    std::unique_ptr<Ui::MainWindow> ui_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pclapp::AnyCloudSession session_;
    QString lastDirectory_;  ///< Where the file dialog opens next time.
};
