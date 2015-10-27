#include "mainwindow.h"

#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QMessageBox>
#include <QKeyEvent>
#include "renderingwidget.h"


MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	renderingwidget_ = new RenderingWidget(this);
//	setCentralWidget(renderingwidget_);

	setGeometry(300, 150, 800, 600);

	CreateActions();
	CreateMenus();
	CreateToolBars();
	CreateStatusBar();
	CreateRenderGroup();
	CreateSliders();

	QVBoxLayout *layout_left = new QVBoxLayout;
	layout_left->addWidget(groupbox_render_);
	layout_left->addStretch(1);
	layout_left->addWidget(slider_layer_);

	QHBoxLayout *layout_main = new QHBoxLayout;
	layout_main->addLayout(layout_left);
	layout_main->addWidget(renderingwidget_);
	layout_main->setStretch(1, 1);
	this->centralWidget()->setLayout(layout_main);

	toolbar_file_->setVisible(false);
}


MainWindow::~MainWindow()
{
}


void MainWindow::CreateActions()
{
	action_new_ = new QAction(QIcon(":/MainWindow/Resources/images/new.png"), tr("&New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon(":/MainWindow/Resources/images/open.png"), tr("&Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));

	action_save_ = new QAction(QIcon(":/MainWindow/Resources/images/save.png"), tr("&Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));
	connect(action_save_, SIGNAL(triggered()), renderingwidget_, SLOT(WriteFrame()));

	action_saveas_ = new QAction(tr("Save &As..."), this);
	action_saveas_->setShortcuts(QKeySequence::SaveAs);
	action_saveas_->setStatusTip(tr("Save the document under a new name"));
//	connect(action_saveas_, SIGNAL(triggered()), imagewidget_, SLOT(SaveAs()));

	action_loadmesh_ = new QAction(tr("readOBJ"), this);
	action_background_ = new QAction(tr("ChangeBackground"), this);
	action_rotatexy_ = new QAction(tr("RotateXY"), this);
	action_rotatexz_ = new QAction(tr("RotateXZ"), this);
	action_rotateyz_ = new QAction(tr("RotateYZ"), this);

	action_fiberprint_ = new QAction(tr("FiberPrint"), this);
	action_simplify_ = new QAction(tr("Simplify"), this);
	action_project_ = new QAction(tr("Project"), this);
	action_setslider_ = new QAction(tr("SetSlider"), this);

	connect(action_loadmesh_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));
	connect(action_background_, SIGNAL(triggered()), renderingwidget_, SLOT(SetBackground()));
	connect(action_rotatexy_, SIGNAL(triggered()), renderingwidget_, SLOT(RotateXY()));
	connect(action_rotatexz_, SIGNAL(triggered()), renderingwidget_, SLOT(RotateXZ()));
	connect(action_rotateyz_, SIGNAL(triggered()), renderingwidget_, SLOT(RotateYZ()));

	connect(action_fiberprint_, SIGNAL(triggered()), renderingwidget_, SLOT(FiberPrintAnalysis()));
	connect(action_simplify_, SIGNAL(triggered()), renderingwidget_, SLOT(SimplifyFrame()));
	connect(action_project_, SIGNAL(triggered()), renderingwidget_, SLOT(ProjectBound()));
	connect(action_setslider_, SIGNAL(triggered()), this, SLOT(SetSlider()));
}


void MainWindow::CreateMenus()
{
	menu_file_ = menuBar()->addMenu(tr("&File"));
	menu_file_->setStatusTip(tr("File menu"));
	menu_file_->addAction(action_new_);
	menu_file_->addAction(action_open_);
	menu_file_->addAction(action_save_);
	menu_file_->addAction(action_saveas_);
}


void MainWindow::CreateToolBars()
{
	toolbar_file_ = addToolBar(tr("File"));
	toolbar_file_->addAction(action_new_);
	toolbar_file_->addAction(action_open_);
	toolbar_file_->addAction(action_save_);

	toolbar_basic_ = addToolBar(tr("Basic"));
	toolbar_basic_->addAction(action_loadmesh_);
	toolbar_basic_->addAction(action_background_);
	toolbar_basic_->addAction(action_rotatexy_);
	toolbar_basic_->addAction(action_rotatexz_);
	toolbar_basic_->addAction(action_rotateyz_);

	toolbar_fiber_ = addToolBar(tr("Fiber"));
	toolbar_fiber_->addAction(action_fiberprint_);
	toolbar_fiber_->addAction(action_simplify_);
	toolbar_fiber_->addAction(action_project_);
	toolbar_fiber_->addAction(action_setslider_);
}


void MainWindow::CreateStatusBar()
{
	label_meshinfo_ = new QLabel(QString("MeshInfo: p: %1 e: %2").arg(0).arg(0));
	label_meshinfo_->setAlignment(Qt::AlignCenter);
	label_meshinfo_->setMinimumSize(label_meshinfo_->sizeHint());

	label_operatorinfo_ = new QLabel();
	label_operatorinfo_->setAlignment(Qt::AlignVCenter);
	
	label_modeinfo_ = new QLabel();

	label_captured_ = new QLabel();

	statusBar()->addWidget(label_meshinfo_);
	connect(renderingwidget_, SIGNAL(meshInfo(int, int)), this, SLOT(ShowMeshInfo(int, int)));
	
	statusBar()->addWidget(label_operatorinfo_);
	connect(renderingwidget_, SIGNAL(operatorInfo(QString)), label_operatorinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_modeinfo_);
	connect(renderingwidget_, SIGNAL(modeInfo(QString)), label_modeinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_captured_);
	connect(renderingwidget_, SIGNAL(CapturedInfo(int)), this, SLOT(ShowCapturedInfo(int)));
}


void MainWindow::CreateRenderGroup()
{
	checkbox_point_ = new QCheckBox(tr("Point"), this);
	connect(checkbox_point_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawPoint(bool)));
	checkbox_point_->setChecked(true);

	checkbox_edge_ = new QCheckBox(tr("Edge"), this);
	connect(checkbox_edge_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawEdge(bool)));

	checkbox_heat_ = new QCheckBox(tr("Heat"), this);
	connect(checkbox_heat_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawHeat(bool)));
	connect(checkbox_heat_, SIGNAL(clicked(bool)), this, SLOT(CheckFace(bool)));

	checkbox_light_ = new QCheckBox(tr("Light"), this);
	connect(checkbox_light_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckLight(bool)));

	checkbox_axes_ = new QCheckBox(tr("Axes"), this);
	connect(checkbox_axes_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawAxes(bool)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);

	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(checkbox_edge_);
	render_layout->addWidget(checkbox_heat_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);
}


void MainWindow::CreateSliders()
{
	slider_layer_ = new QSlider(Qt::Horizontal);
	slider_layer_->setMinimum(0);
	slider_layer_->setMaximum(10);

	connect( slider_layer_, SIGNAL(valueChanged(int)), 
				renderingwidget_, SLOT(PrintLayer(int)) );
}


void MainWindow::keyPressEvent(QKeyEvent *e)
{

}


void MainWindow::keyReleaseEvent(QKeyEvent *e)
{

}


void MainWindow::ShowMeshInfo(int npoint, int nedge)
{
	label_meshinfo_->setText(QString("MeshInfo: p: %1 e: %2").arg(npoint).arg(nedge));
}


void MainWindow::ShowCapturedInfo(int id)
{
	label_captured_->setText(QString("Captured: %1").arg(id));
}


void MainWindow::OpenFile()
{

}


void MainWindow::ShowAbout()
{
	QMessageBox::information(this, "About QtMeshFrame-1.0.1",

		QString("<h3>This MeshFrame provides some operations about *.obj files sunch as") +
		" IO, render with points , edges, triangles or textures and some interactions with mouse."
		" A fix light source is provided for you."
		"This is a basic and raw frame for handling meshes. The mesh is of half_edge struct.\n"
		"Please contact" "<font color=blue> wkcagd@mail.ustc.edu.cn<\font><font color=black>, Kang Wang if you has any questions.<\font><\h3>"
		,
		QMessageBox::Ok);
}


void MainWindow::SetSlider()
{
	bool ok;
	int max_range = QInputDialog::getInt(
					this, 
					tr("Maximum Layers"), 
					tr("Please input the maximum layers"), 
					slider_layer_->maximum(), 0, 100, 1, &ok);
	if (ok)
	{
		slider_layer_->setMaximum(max_range);

	}
}
