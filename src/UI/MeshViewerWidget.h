#pragma once
#include "../Base/ply.h"
#include "../Scene/meshing.h"
#include "../Scene/Reconstruction.h"
#include "Colormap.h"
#include "PointLinePainter.h"
#include <QOpenGLFunctions_3_2_Core>
#include <QtCore>
#include <QtOpenGL>
class MeshViewerWidget : public QOpenGLWidget , protected QOpenGLFunctions_3_2_Core
{
    Q_OBJECT
public:
    const float kInitNearPlane = 1.0f;
    const float kMinNearPlane = 1e-3f;
    const float kMaxNearPlane = 1e5f;
    const float kNearPlaneScaleSpeed = 0.02f;
    const float kFarPlane = 1e5f;
    const float kInitFocusDistance = 100.0f;
    const float kMinFocusDistance = 1e-7f;
    const float kMaxFocusDistance = 1e12f;
    const float kFieldOfView = 25.0f;
    const float kFocusSpeed = 2.0f;
    const float kInitPointSize = 1.5f;
    const float kMinPointSize = 0.5f;
    const float kMaxPointSize = 100.0f;
    const float kPointScaleSpeed = 0.1f;
    const float kInitImageSize = 0.2f;
    const float kMinImageSize = 1e-6f;
    const float kMaxImageSize = 1e3f;
    const float kImageScaleSpeed = 0.1f;
    const int kDoubleClickInterval = 250;

    MeshViewerWidget(QWidget* parent);
    void SetDatabase(Database* database);

    void ReloadReconstruction();
    void ClearReconstruction();

    void ReloadReconstructionWithFeedback(bool showMeshline = false , bool showHeatmap = false , bool showGuidenceline = false);

    int GetProjectionType() const;

    // Takes ownwership of the colormap objects.
    void SetPointColormap(PointColormapBase* colormap);
    void SetImageColormap(ImageColormapBase* colormap);

    void UpdateMovieGrabber();

    void EnableCoordinateGrid();
    void DisableCoordinateGrid();

    void ChangeFocusDistance(float delta);
    void ChangeNearPlane(float delta);
    void ChangePointSize(float delta);
    void ChangeCameraSize(float delta);

    void RotateView(float x , float y , float prev_x , float prev_y);
    void TranslateView(float x , float y , float prev_x , float prev_y);

    void ResetView();

    QMatrix4x4 ModelViewMatrix() const;
    void SetModelViewMatrix(const QMatrix4x4& matrix);

    void SelectObject(int x , int y);
    void SelectMoviewGrabberView(size_t view_idx);

    QImage GrabImage();
    void GrabMovie();

    void ShowPointInfo(point3D_t point3D_id);
    void ShowImageInfo(image_t image_id);

    float PointSize() const;
    float ImageSize() const;
    void SetPointSize(float point_size);
    void SetImageSize(float image_size);

    void SetBackgroundColor(float r , float g , float b);

    // Copy of current scene data that is displayed
    std::shared_ptr<Reconstruction> reconstruction;
    tbb::concurrent_unordered_map<camera_t , Camera> cameras;
    tbb::concurrent_unordered_map<image_t , Image> images;
    tbb::concurrent_unordered_map<point3D_t , Point3D> points3D;
    std::vector<image_t> reg_image_ids;

    std::shared_ptr<PlyMesh> mesh;

    //Feedback Rendering data
    std::shared_ptr<RenderingData> rendering_data;

    QString status_label;
    void UpdateStatus(const QString& status);
signals:
    void statusUpdate(const QString& status);

protected:
    void initializeGL() override;
    void resizeGL(int width , int height) override;
    void paintGL() override;

private:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    void SetupPainters();
    void SetupView();

    void Upload();
    void UploadCoordinateGridData();
    void UploadPointData(bool selection_mode = false);
    void UploadPointConnectionData();
    void UploadImageData(bool selection_mode = false);
    void UploadImageConnectionData();
    void UploadMovieGrabberData();

public:
    //Feedback
    void UploadMeshData(bool showMeshline = false); // 上传mesh数据
    // --->上传反馈数据，默认显示表面mesh，不显示Heatmap以及Guideline
    void UploadGuidenceData(bool showHeatmap = false , bool showGuideline = false);

private:
    void ComposeProjectionMatrix();

    float ZoomScale() const;
    float AspectRatio() const;
    float OrthographicWindowExtent() const;

    Eigen::Vector3f PositionToArcballVector(float x , float y) const;


    QMatrix4x4 model_view_matrix_;
    QMatrix4x4 projection_matrix_;

    LinePainter coordinate_axes_painter_;
    LinePainter coordinate_grid_painter_;

    PointPainter point_painter_;
    LinePainter point_connection_painter_;

    LinePainter image_line_painter_;
    TrianglePainter image_triangle_painter_;
    LinePainter image_connection_painter_;

    LinePainter movie_grabber_path_painter_;
    LinePainter movie_grabber_line_painter_;
    TrianglePainter movie_grabber_triangle_painter_;

    Database* database = nullptr;

    //Feedback Painter
    TrianglePainter mesh_triangle_painter_;
    LinePainter mesh_line_painter_;
    PointPainter marker_point_painter_;//紫色引导点(交点)
    LinePainter guide_line_painter_;//引导线
    TrianglePainter base_plane_painter_;//基面
    PointPainter heatmap_point_painter_;//热力图点

    std::unique_ptr<PointColormapBase> point_colormap_;
    std::unique_ptr<ImageColormapBase> image_colormap_;

    bool mouse_is_pressed_;
    QTimer mouse_press_timer_;
    QPoint prev_mouse_pos_;

    float focus_distance_;

    std::vector<std::pair<size_t , char>> selection_buffer_;
    image_t selected_image_id_;
    point3D_t selected_point3D_id_;
    size_t selected_movie_grabber_view_;

    bool coordinate_grid_enabled_;

    // Size of points (dynamic): does not require re-uploading of points.
    float point_size_;
    // Size of image models (not dynamic): requires re-uploading of image models.
    float image_size_;
    // Near clipping plane.
    float near_plane_;

    float background_color_[3];

};
