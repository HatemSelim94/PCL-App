#include "mainwindow.h"

#include <QActionGroup>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <pcl/common/colors.h>

#include <type_traits>
#include <variant>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include "language.h"
#include "ui_mainwindow.h"

namespace {

constexpr double kPointSize = 2.0;  // in pixels; 1 is hard to see on dense screens

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

    // The Qt widget owns the OpenGL context, so the visualizer must not open a
    // window of its own. It gets a render window that draws into whatever
    // context is current, and that window is then handed to the widget.
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer_ = pcl::make_shared<pcl::visualization::PCLVisualizer>(renderer, renderWindow, "viewer",
                                                                  /*create_interactor=*/false);
    viewer_->setShowFPS(false);

    ui_->viewWidget->setRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui_->viewWidget->interactor(), ui_->viewWidget->renderWindow());
    viewer_->resetCamera();
    refreshView();

    setupLanguageMenu();
    updateControls();
}

void MainWindow::changeEvent(QEvent* event) {
    if (event->type() == QEvent::LanguageChange) {
        ui_->retranslateUi(this);
        updateFileLabel();
    }
    QMainWindow::changeEvent(event);
}

void MainWindow::setupLanguageMenu() {
    // One checkable action per language plus "system default"; the group
    // keeps exactly one checked. The code travels in the action's data.
    auto* group = new QActionGroup(this);
    const QString saved = pclapp::app::savedLanguageCode();

    auto addLanguage = [&](const QString& code, const QString& name) {
        QAction* action = ui_->menuLanguage->addAction(name);
        action->setCheckable(true);
        action->setData(code);
        action->setChecked(code == saved);
        group->addAction(action);
    };
    addLanguage(QString(), tr("System default"));
    ui_->menuLanguage->addSeparator();
    for (const auto& language : pclapp::app::availableLanguages()) {
        addLanguage(language.code, language.nativeName);
    }

    connect(group, &QActionGroup::triggered, this, [](QAction* action) {
        const QString code = action->data().toString();
        pclapp::app::saveLanguageCode(code);
        pclapp::app::applyLanguage(code);
    });
}

// Defined here, where Ui::MainWindow is a complete type for unique_ptr.
MainWindow::~MainWindow() = default;

template <typename Action>
void MainWindow::withSession(Action&& action) {
    std::visit(
        [&](auto& session) {
            if constexpr (!std::is_same_v<std::decay_t<decltype(session)>, std::monostate>) {
                try {
                    action(session);
                } catch (const std::exception& error) {
                    reportError(error);
                }
            }
        },
        session_);
    updateControls();
}

void MainWindow::on_actionOpen_triggered() {
    on_loadButton_clicked();
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About PCL App"),
                       tr("<b>PCL App</b><br>"
                          "Loads, downsamples and clusters point clouds.<br><br>"
                          "The geometry algorithms come from the Point Cloud Library (PCL); "
                          "the 3D view is VTK in a Qt window.<br><br>"
                          "Hatem Selim, 2021&ndash;2026<br>"
                          "<a href=\"https://github.com/HatemSelim94/PCL-App\">"
                          "github.com/HatemSelim94/PCL-App</a>"));
}

void MainWindow::on_loadButton_clicked() {
    const QString file = QFileDialog::getOpenFileName(
        this, tr("Select a point cloud"), lastDirectory_, tr("Point clouds (*.pcd *.txt *.xyz)"));
    if (file.isEmpty()) return;
    lastDirectory_ = QFileInfo(file).absolutePath();

    const auto kind =
        ui_->xyziRadio->isChecked() ? pclapp::PointKind::Xyzi : pclapp::PointKind::Xyz;
    try {
        session_ = pclapp::openCloud(toPath(file), kind);
    } catch (const std::exception& error) {
        reportError(error);  // the previous session, if any, stays loaded
        return;
    }

    loadedFile_ = QFileInfo(file).fileName();
    withSession([&](auto& session) {
        updateFileLabel();
        statusBar()->showMessage(
            tr("Loaded %1 points from %2").arg(session.source()->size()).arg(file));
        showCloud(session.source(), "source");
        viewer_->resetCamera();  // frame the new cloud; later views keep the camera
        refreshView();
    });
}

void MainWindow::on_showSourceButton_clicked() {
    withSession([&](auto& session) { showCloud(session.source(), "source"); });
}

void MainWindow::on_downsampleButton_clicked() {
    withSession([&](auto& session) {
        session.applyVoxelFilter(voxelParamsFromUi());
        statusBar()->showMessage(tr("Downsampled %1 points to %2")
                                     .arg(session.source()->size())
                                     .arg(session.processed()->size()));
        showCloud(session.processed(), "processed");
    });
}

void MainWindow::on_showDownsampledButton_clicked() {
    withSession([&](auto& session) { showCloud(session.processed(), "processed"); });
}

void MainWindow::on_clusterButton_clicked() {
    withSession([&](auto& session) {
        session.extractClusters(clusterParamsFromUi());
        const int count = static_cast<int>(session.clusters().size());
        statusBar()->showMessage(tr("Found %n cluster(s)", nullptr, count));  // plural per language
        showClusters(session.clusters());
    });
}

void MainWindow::on_showClustersButton_clicked() {
    withSession([&](auto& session) { showClusters(session.clusters()); });
}

void MainWindow::on_saveButton_clicked() {
    withSession([&](auto& session) {
        const auto written = session.saveProcessed();
        statusBar()->showMessage(tr("Saved downsampled cloud to %1").arg(toQString(written)));
    });
}

void MainWindow::on_convertButton_clicked() {
    withSession([&](auto& session) {
        const auto written = session.convertSource();
        statusBar()->showMessage(tr("Converted to %1").arg(toQString(written)));
    });
}

template <typename PointT>
void MainWindow::showCloud(const pclapp::CloudPtr<PointT>& cloud, const std::string& id) {
    viewer_->removeAllPointClouds();
    viewer_->addPointCloud<PointT>(cloud, id);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                              kPointSize, id);
    refreshView();
}

template <typename PointT>
void MainWindow::showClusters(const std::vector<pclapp::CloudPtr<PointT>>& clusters) {
    if (clusters.empty()) {
        statusBar()->showMessage(tr("No clusters found with these parameters."));
        return;
    }

    viewer_->removeAllPointClouds();
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        // Glasbey colours are chosen to be maximally distinguishable from each other.
        const pcl::RGB color =
            pcl::GlasbeyLUT::at(static_cast<unsigned>(i % pcl::GlasbeyLUT::size()));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> handler(clusters[i], color.r,
                                                                         color.g, color.b);
        const std::string id = "cluster" + std::to_string(i);
        viewer_->addPointCloud<PointT>(clusters[i], handler, id);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  kPointSize, id);
    }
    refreshView();
}

pclapp::VoxelGridParams MainWindow::voxelParamsFromUi() const {
    return {static_cast<float>(ui_->leafSizeSpin->value())};
}

pclapp::EuclideanClusterParams MainWindow::clusterParamsFromUi() const {
    return {static_cast<float>(ui_->toleranceSpin->value()), ui_->minPointsSpin->value(),
            ui_->maxPointsSpin->value()};
}

void MainWindow::reportError(const std::exception& error) {
    QMessageBox::warning(this, tr("Operation failed"), QString::fromUtf8(error.what()));
}

void MainWindow::updateFileLabel() {
    std::visit(
        [&](const auto& session) {
            if constexpr (std::is_same_v<std::decay_t<decltype(session)>, std::monostate>) {
                ui_->fileLabel->setText(tr("No file loaded"));
            } else {
                ui_->fileLabel->setText(tr("%1\n%2 points, %3")
                                            .arg(loadedFile_)
                                            .arg(session.source()->size())
                                            .arg(ui_->xyziRadio->isChecked()
                                                     ? QStringLiteral("XYZI")
                                                     : QStringLiteral("XYZ")));
            }
        },
        session_);
}

void MainWindow::refreshView() {
    ui_->viewWidget->renderWindow()->Render();
}

void MainWindow::updateControls() {
    const bool loaded = !std::holds_alternative<std::monostate>(session_);
    const bool hasClusters = std::visit(
        [](const auto& session) {
            if constexpr (std::is_same_v<std::decay_t<decltype(session)>, std::monostate>) {
                return false;
            } else {
                return !session.clusters().empty();
            }
        },
        session_);

    for (QWidget* widget :
         {static_cast<QWidget*>(ui_->showSourceButton), static_cast<QWidget*>(ui_->downsampleGroup),
          static_cast<QWidget*>(ui_->clusterGroup), static_cast<QWidget*>(ui_->saveButton),
          static_cast<QWidget*>(ui_->convertButton)}) {
        widget->setEnabled(loaded);
    }
    ui_->showClustersButton->setEnabled(loaded && hasClusters);
}
