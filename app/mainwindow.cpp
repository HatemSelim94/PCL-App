#include "mainwindow.h"

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <pcl/common/colors.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <type_traits>
#include <variant>

#include "ui_mainwindow.h"

namespace {

// char16_t keeps non-ASCII file names intact on every platform.
std::filesystem::path toPath(const QString& text) {
    return std::filesystem::path(text.toStdU16String());
}

QString toQString(const std::filesystem::path& path) {
    return QString::fromStdU16String(path.u16string());
}

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui_(std::make_unique<Ui::MainWindow>()) {
    ui_->setupUi(this);
    setWindowTitle(tr("PCL Application"));

    // The Qt widget owns the OpenGL context, so the visualizer must not open a
    // window of its own. It gets a render window that draws into whatever
    // context is current, and that window is then handed to the widget.
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer_ = pcl::make_shared<pcl::visualization::PCLVisualizer>(
        renderer, renderWindow, "viewer", /*create_interactor=*/false);

    ui_->qvtkWidget->setRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui_->qvtkWidget->interactor(), ui_->qvtkWidget->renderWindow());
    viewer_->resetCamera();
    refreshView();
}

// Defined here, where Ui::MainWindow is a complete type for unique_ptr.
MainWindow::~MainWindow() = default;

template <typename Action>
void MainWindow::withSession(Action&& action) {
    std::visit(
        [&](auto& session) {
            if constexpr (std::is_same_v<std::decay_t<decltype(session)>, std::monostate>) {
                QMessageBox::information(this, tr("No data"), tr("Please load a file first."));
            } else {
                try {
                    action(session);
                } catch (const std::exception& error) {
                    reportError(error);
                }
            }
        },
        session_);
}

void MainWindow::on_loadFile_clicked() {
    const QString file = QFileDialog::getOpenFileName(
        this, tr("Select a point cloud"), QString(), tr("Point clouds (*.txt *.xyz *.pcd)"));
    if (file.isEmpty()) return;

    const auto kind = ui_->XYZIData->isChecked() ? pclapp::PointKind::Xyzi : pclapp::PointKind::Xyz;
    try {
        session_ = pclapp::openCloud(toPath(file), kind);
    } catch (const std::exception& error) {
        reportError(error);  // the previous session, if any, stays loaded
        return;
    }

    withSession([&](auto& session) {
        statusBar()->showMessage(
            tr("Loaded %1 points from %2").arg(session.source()->size()).arg(file));
        showCloud(session.source(), "source");
        viewer_->resetCamera();  // frame the new cloud; later views keep the camera
        refreshView();
    });
}

void MainWindow::on_showPointCloud_clicked() {
    withSession([&](auto& session) { showCloud(session.source(), "source"); });
}

void MainWindow::on_Filter_clicked() {
    withSession([&](auto& session) {
        session.applyVoxelFilter(voxelParamsFromUi());
        statusBar()->showMessage(tr("Voxel filter: %1 -> %2 points")
                                     .arg(session.source()->size())
                                     .arg(session.processed()->size()));
    });
}

void MainWindow::on_showFilteredCloud_clicked() {
    withSession([&](auto& session) { showCloud(session.processed(), "processed"); });
}

void MainWindow::on_Cluster_clicked() {
    withSession([&](auto& session) {
        session.extractClusters(clusterParamsFromUi());
        statusBar()->showMessage(tr("Found %1 clusters").arg(session.clusters().size()));
    });
}

void MainWindow::on_showClusters_clicked() {
    withSession([&](auto& session) { showClusters(session.clusters()); });
}

void MainWindow::on_saveFilteredCloud_clicked() {
    withSession([&](auto& session) {
        const auto written = session.saveProcessed();
        statusBar()->showMessage(tr("Saved filtered cloud to %1").arg(toQString(written)));
    });
}

void MainWindow::on_ConvertFile_clicked() {
    withSession([&](auto& session) {
        const auto written = session.convertSource();
        statusBar()->showMessage(tr("Converted to %1").arg(toQString(written)));
    });
}

template <typename PointT>
void MainWindow::showCloud(const pclapp::CloudPtr<PointT>& cloud, const std::string& id) {
    viewer_->removeAllPointClouds();
    viewer_->addPointCloud<PointT>(cloud, id);
    refreshView();
}

template <typename PointT>
void MainWindow::showClusters(const std::vector<pclapp::CloudPtr<PointT>>& clusters) {
    if (clusters.empty()) {
        statusBar()->showMessage(tr("No clusters yet. Run clustering first."));
        return;
    }

    viewer_->removeAllPointClouds();
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        // Glasbey colours are chosen to be maximally distinguishable from each other.
        const pcl::RGB color =
            pcl::GlasbeyLUT::at(static_cast<unsigned>(i % pcl::GlasbeyLUT::size()));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> handler(
            clusters[i], color.r, color.g, color.b);
        viewer_->addPointCloud<PointT>(clusters[i], handler, "cluster" + std::to_string(i));
    }
    refreshView();
}

void MainWindow::refreshView() {
    ui_->qvtkWidget->renderWindow()->Render();
}

pclapp::VoxelGridParams MainWindow::voxelParamsFromUi() const {
    return {static_cast<float>(ui_->voxelLeafSize->value())};
}

pclapp::EuclideanClusterParams MainWindow::clusterParamsFromUi() const {
    return {static_cast<float>(ui_->distanceTolerance->value()),
            ui_->minClusterPoints->value(),
            ui_->maxClusterPoints->value()};
}

void MainWindow::reportError(const std::exception& error) {
    QMessageBox::warning(this, tr("Operation failed"), QString::fromUtf8(error.what()));
}
