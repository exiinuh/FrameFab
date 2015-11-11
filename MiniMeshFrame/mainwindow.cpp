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
	CreateEditGroup();
	CreateScaleGroup();

	QVBoxLayout *layout_left = new QVBoxLayout;
	layout_left->addWidget(groupbox_render_);
	layout_left->addWidget(groupbox_edit_);
	layout_left->addWidget(groupbox_scale_);
	layout_left->addStretch(1);

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
	action_new_ = new QAction(QIcon(":/Resources/images/new.png"), tr("New"), this);
	action_new_->setShortcut(QKeySequence::New);
	action_new_->setStatusTip(tr("Create a new file"));

	action_open_ = new QAction(QIcon(":/MainWindow/Resources/images/open.png"), tr("Open..."), this);
	action_open_->setShortcuts(QKeySequence::Open);
	action_open_->setStatusTip(tr("Open an existing file"));
	connect(action_open_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));

	action_save_ = new QAction(QIcon(":/MainWindow/Resources/images/save.png"), tr("Save"), this);
	action_save_->setShortcuts(QKeySequence::Save);
	action_save_->setStatusTip(tr("Save the document to disk"));
	connect(action_save_, SIGNAL(triggered()), renderingwidget_, SLOT(WriteFrame()));
	
	action_saveas_ = new QAction(tr("Save As..."), this);
	action_saveas_->setShortcuts(QKeySequence::SaveAs);
	action_saveas_->setStatusTip(tr("Save the document under a new name"));
//	connect(action_saveas_, SIGNAL(triggered()), imagewidget_, SLOT(SaveAs()));

	action_loadmesh_ = new QAction(tr("readOBJ"), this);
	action_background_ = new QAction(tr("ChangeBackground"), this);

	action_fiberprint_ = new QAction(tr("FiberPrint"), this);
	action_simplify_ = new QAction(tr("Simplify"), this);
	action_project_ = new QAction(tr("Project"), this);

	connect(action_loadmesh_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));
	connect(action_background_, SIGNAL(triggered()), renderingwidget_, SLOT(SetBackground()));

	connect(action_fiberprint_, SIGNAL(triggered()), renderingwidget_, SLOT(FiberPrintAnalysis()));
	connect(action_simplify_, SIGNAL(triggered()), renderingwidget_, SLOT(SimplifyFrame()));
	connect(action_project_, SIGNAL(triggered()), renderingwidget_, SLOT(ProjectBound()));
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

	toolbar_fiber_ = addToolBar(tr("Fiber"));
	toolbar_fiber_->addAction(action_fiberprint_);
	toolbar_fiber_->addAction(action_simplify_);
	toolbar_fiber_->addAction(action_project_);
}


void MainWindow::CreateStatusBar()
{
	label_meshinfo_ = new QLabel(QString("MeshInfo: p: %1 e: %2").arg(0).arg(0));
	label_meshinfo_->setAlignment(Qt::AlignCenter);
	label_meshinfo_->setMinimumSize(label_meshinfo_->sizeHint());

	label_operatorinfo_ = new QLabel();
	label_operatorinfo_->setAlignment(Qt::AlignVCenter);
	
	label_modeinfo_ = new QLabel();

	label_capture_ = new QLabel();

	statusBar()->addWidget(label_meshinfo_);
	connect(renderingwidget_, SIGNAL(meshInfo(int, int)), this, SLOT(ShowMeshInfo(int, int)));
	
	statusBar()->addWidget(label_operatorinfo_);
	connect(renderingwidget_, SIGNAL(operatorInfo(QString)), label_operatorinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_modeinfo_);
	connect(renderingwidget_, SIGNAL(modeInfo(QString)), label_modeinfo_, SLOT(setText(QString)));

	statusBar()->addWidget(label_capture_);
	connect(renderingwidget_, SIGNAL(CapturedVert(int)), this, SLOT(ShowCapturedVert(int)));
	connect(renderingwidget_, SIGNAL(CapturedEdge(int, double)), this, SLOT(ShowCapturedEdge(int, double)));
}


void MainWindow::CreateRenderGroup()
{
	checkbox_point_ = new QCheckBox(tr("Point"), this);
	connect(checkbox_point_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawPoint(bool)));
	checkbox_point_->setChecked(true);
	
	radiobutton_edge_ = new QRadioButton(tr("Edge"), this);
	connect(radiobutton_edge_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_heat_ = new QRadioButton(tr("Heat"), this);
	connect(radiobutton_heat_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_cut_ = new QRadioButton(tr("Cut"), this);
	connect(radiobutton_cut_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	slider_layer_ = new QSlider(Qt::Horizontal);
	slider_layer_->setMinimum(0);
	slider_layer_->setMaximum(10);
	connect(slider_layer_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintLayer(int)));

	radiobutton_bulk_ = new QRadioButton(tr("Bulk"), this);
	connect(radiobutton_bulk_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_order_ = new QRadioButton(tr("Order"), this);
	connect(radiobutton_order_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	slider_order_ = new QSlider(Qt::Horizontal);
	slider_order_->setMinimum(0);
	slider_order_->setMaximum(10);
	connect(slider_order_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintOrder(int)));

	radiobutton_none_ = new QRadioButton(tr("None"), this);
	radiobutton_none_->setVisible(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;

	connect(this, SIGNAL(EdgeMode(int)), renderingwidget_, SLOT(CheckEdgeMode(int)));

	checkbox_light_ = new QCheckBox(tr("Light"), this);
	connect(checkbox_light_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckLight(bool)));

	checkbox_axes_ = new QCheckBox(tr("Axes"), this);
	connect(checkbox_axes_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawAxes(bool)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);

	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(radiobutton_edge_);
	render_layout->addWidget(radiobutton_heat_);
	render_layout->addWidget(radiobutton_cut_);
	render_layout->addWidget(slider_layer_);
	render_layout->addWidget(radiobutton_bulk_);
	render_layout->addWidget(radiobutton_order_);
	render_layout->addWidget(slider_order_);
	render_layout->addWidget(radiobutton_none_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);
}


void MainWindow::CreateEditGroup()
{
	pushbutton_rotatexy_ = new QPushButton(tr("RotateXY"), this);
	pushbutton_rotatexz_ = new QPushButton(tr("RotateXZ"), this);
	pushbutton_rotateyz_ = new QPushButton(tr("RotateYZ"), this);
	connect(pushbutton_rotatexy_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXY()));
	connect(pushbutton_rotatexz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXZ()));
	connect(pushbutton_rotateyz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateYZ()));

	groupbox_edit_ = new QGroupBox(tr("Edit"), this);

	QVBoxLayout* edit_layout = new QVBoxLayout(groupbox_edit_);
	edit_layout->addWidget(pushbutton_rotatexy_);
	edit_layout->addWidget(pushbutton_rotatexz_);
	edit_layout->addWidget(pushbutton_rotateyz_);
}


void MainWindow::CreateScaleGroup()
{
	slider_scale_ = new QSlider(Qt::Horizontal);

	groupbox_scale_ = new QGroupBox(tr("Scale"), this);

	QVBoxLayout* scale_layout = new QVBoxLayout(groupbox_scale_);
	scale_layout->addWidget(slider_scale_);
}


void MainWindow::keyPressEvent(QKeyEvent *e)
{

}


void MainWindow::keyReleaseEvent(QKeyEvent *e)
{

}


void MainWindow::OpenFile()
{

}


void MainWindow::ShowMeshInfo(int npoint, int nedge)
{
	label_meshinfo_->setText(QString("MeshInfo: p: %1 e: %2").arg(npoint).arg(nedge));
}


void MainWindow::ShowCapturedVert(int id)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured vertex: %1").arg(id));
	}
	else
	{
		label_capture_->setText(QString(""));
	}
}


void MainWindow::ShowCapturedEdge(int id, double len)
{
	if (id != -1)
	{
		label_capture_->setText(QString("Captured edge: %1  Length: %2").arg(id).arg(len));
	}
	else
	{
		label_capture_->setText(QString(""));
	}
}


void MainWindow::EdgeModeChange()
{
	if (sender() == radiobutton_edge_)
	{
		if (edge_render_ != EDGE)
		{
			radiobutton_edge_->setChecked(true);
			edge_render_ = EDGE;
			emit(EdgeMode(EDGE));
			return;
		}
	}
	else
	if (sender() == radiobutton_heat_)
	{
		if (edge_render_ != HEAT)
		{
			radiobutton_heat_->setChecked(true);
			edge_render_ = HEAT;
			emit(EdgeMode(HEAT));
			return;
		}
	}
	else
	if (sender() == radiobutton_cut_)
	{
		if (edge_render_ != CUT)
		{
			radiobutton_cut_->setChecked(true);
			edge_render_ = CUT;
			emit(EdgeMode(CUT));
			return;
		}
	}
	else
	if (sender() == radiobutton_bulk_)
	{
		if (edge_render_ != BULK)
		{
			radiobutton_bulk_->setChecked(true);
			edge_render_ = BULK;
			emit(EdgeMode(BULK));
			return;
		}
	}
	else
	if (sender() == radiobutton_order_)
	{
		if (edge_render_ != ORDER)
		{
			radiobutton_order_->setChecked(true);
			edge_render_ = ORDER;
			emit(EdgeMode(ORDER));
			return;
		}
	}

	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;
	emit(EdgeMode(NONE));
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
