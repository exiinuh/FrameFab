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

	setGeometry(200, 150, 1000, 700);

	CreateActions();
	CreateMenus();
	CreateToolBars();
	CreateLabels();
	CreateLineEdits();
	CreateCheckBoxes();
	CreateRadioButtons();
	CreatePushButtons();
	CreateToolButtons();
	CreateSliders();
	CreateGroups();

	connect(renderingwidget_, SIGNAL(Reset()), this, SLOT(Reset()));

	QVBoxLayout *layout_left = new QVBoxLayout; 
	layout_left->addWidget(groupbox_render_);
	layout_left->addWidget(groupbox_edge_);
	layout_left->addWidget(groupbox_orderdisplay_);
	layout_left->addWidget(groupbox_ordersettings_);
	layout_left->addWidget(groupbox_edit_);
	layout_left->addWidget(groupbox_scale_);
	layout_left->addWidget(groupbox_separator_);
	layout_left->addStretch(1);

	QVBoxLayout *layout_right = new QVBoxLayout;
	layout_right->addWidget(groupbox_fiber_);
	layout_right->addWidget(groupbox_para_);
	layout_right->addStretch(1);

	QHBoxLayout *layout_main = new QHBoxLayout;
	layout_main->addLayout(layout_left);
	layout_main->addWidget(renderingwidget_);
	layout_main->setStretch(1, 1);
	layout_main->addLayout(layout_right);
	this->centralWidget()->setLayout(layout_main);

	toolbar_file_->setVisible(false);

	Reset();
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

	connect(action_loadmesh_, SIGNAL(triggered()), renderingwidget_, SLOT(ReadFrame()));
	connect(action_background_, SIGNAL(triggered()), renderingwidget_, SLOT(SetBackground()));
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
}


void MainWindow::CreateLabels()
{
	label_meshinfo_ = new QLabel(QString("MeshInfo: p: %1 e: %2").arg(0).arg(0));
	label_meshinfo_->setAlignment(Qt::AlignCenter);
	label_meshinfo_->setMinimumSize(label_meshinfo_->sizeHint());

	label_operatorinfo_ = new QLabel(QString("Scale: 1.0"));
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


	label_radius_		= new QLabel(QString("Radius(mm): "));
	label_density_		= new QLabel(QString("Density(Ton/mm^3): "));
	label_g_			= new QLabel(QString("Gravity(m/s^2): "));
	label_youngsmodulus_= new QLabel(QString("Young's modulus(MPa): "));
	label_shearmodulus_ = new QLabel(QString("Shear modulus(MPa): "));

	label_penalty_	= new QLabel(QString("ADMM penalty: "));
	label_Dtol_		= new QLabel(QString("ADMM D tolerance: "));
	label_pritol_	= new QLabel(QString("ADMM primal tolerance: "));
	label_dualtol_	= new QLabel(QString("ADMM dual tolerance: "));
	label_alpha_	= new QLabel(QString("TSP alpha: "));
	label_beta_		= new QLabel(QString("TSP beta: "));
	label_gamma_	= new QLabel(QString("TSP gamma: "));
}


void MainWindow::CreateLineEdits()
{
	line_radius_		= new QLineEdit(tr("0.4"), this);
	line_density_		= new QLineEdit(tr("1210 * 1e-12"), this);
	line_g_				= new QLineEdit(tr("-9806.33"), this);
	line_youngsmodulus_	= new QLineEdit(tr("1100"), this);
	line_shearmodulus_	= new QLineEdit(tr("1032"), this);

	line_penalty_	= new QLineEdit(tr("100"), this);
	line_Dtol_		= new QLineEdit(tr("0.1"), this);
	line_pritol_	= new QLineEdit(tr("0.001"), this);
	line_dualtol_	= new QLineEdit(tr("0.001"), this);

	line_alpha_		= new QLineEdit(tr("1.0"), this);
	line_beta_		= new QLineEdit(tr("10000.0"), this);
	line_gamma_		= new QLineEdit(tr("100.0"), this);
}


void MainWindow::CreateCheckBoxes()
{
	checkbox_point_ = new QCheckBox(tr("Point"), this);
	connect(checkbox_point_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawPoint(bool)));
	checkbox_point_->setChecked(true);

	checkbox_light_ = new QCheckBox(tr("Light"), this);
	connect(checkbox_light_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckLight(bool)));

	checkbox_axes_ = new QCheckBox(tr("Axes"), this);
	connect(checkbox_axes_, SIGNAL(clicked(bool)), renderingwidget_, SLOT(CheckDrawAxes(bool)));
}


void MainWindow::CreateRadioButtons()
{
	radiobutton_heat_ = new QRadioButton(tr("Heat"), this);
	connect(radiobutton_heat_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_cut_ = new QRadioButton(tr("Cut"), this);
	connect(radiobutton_cut_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_bulk_ = new QRadioButton(tr("Bulk"), this);
	connect(radiobutton_bulk_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_order_ = new QRadioButton(tr("Order"), this);
	connect(radiobutton_order_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	radiobutton_none_ = new QRadioButton(tr("None"), this);
	radiobutton_none_->setVisible(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;
}


void MainWindow::CreatePushButtons()
{
	pushbutton_rotatexy_ = new QPushButton(tr("RotateXY"), this);
	pushbutton_rotatexz_ = new QPushButton(tr("RotateXZ"), this);
	pushbutton_rotateyz_ = new QPushButton(tr("RotateYZ"), this);
	connect(pushbutton_rotatexy_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXY()));
	connect(pushbutton_rotatexz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateXZ()));
	connect(pushbutton_rotateyz_, SIGNAL(clicked()), renderingwidget_, SLOT(RotateYZ()));

	pushbutton_orientation_ = new QPushButton(tr("Change\norientation"), this);
	connect(pushbutton_orientation_, SIGNAL(clicked()), renderingwidget_, SLOT(ChangeOrientation()));

	pushbutton_simplify_ = new QPushButton(tr("Simplify"), this);
	connect(pushbutton_simplify_, SIGNAL(clicked()), renderingwidget_, SLOT(SimplifyFrame()));

	pushbutton_fiberprint_ = new QPushButton(tr("Fiber print"), this);
	pushbutton_project_ = new QPushButton(tr("Project"), this);
	connect(pushbutton_fiberprint_, SIGNAL(clicked()), this, SLOT(GetParameters())); 
	connect(this, SIGNAL(SendParameters(double, double, double, double, double, double, double, 
		double, double, double, double, double)), renderingwidget_, SLOT(FiberPrintAnalysis(double, 
		double, double, double, double, double, double, double, double, double, double, double)));
	connect(pushbutton_project_, SIGNAL(clicked()), renderingwidget_, SLOT(ProjectBound()));
}


void MainWindow::CreateToolButtons()
{
	toolbutton_addedge_ = new QToolButton();
	toolbutton_addedge_->setText(tr("Insert\nedge"));
	toolbutton_addedge_->setMaximumSize(100, 50);
	connect(toolbutton_addedge_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToAddEdge()));

	toolbutton_choosebound_ = new QToolButton();
	toolbutton_choosebound_->setText(tr("Choose frame\nboundary"));
	toolbutton_choosebound_->setMaximumSize(200, 150);
	connect(toolbutton_choosebound_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToChooseBound()));

	toolbutton_setstart_ = new QToolButton();
	toolbutton_setstart_->setText(tr("Set as\nstart edge"));
	toolbutton_setstart_->setMaximumSize(84, 50);
	connect(toolbutton_setstart_, SIGNAL(clicked()), renderingwidget_, SLOT(SwitchToSetStart()));

	connect(renderingwidget_, SIGNAL(AddEdgePressed(bool)), this, SLOT(AddEdgeClicked(bool)));
	connect(renderingwidget_, SIGNAL(ChooseBoundPressed(bool)), this, SLOT(ChooseBoundClicked(bool)));
	connect(renderingwidget_, SIGNAL(SetStartPressed(bool)), this, SLOT(SetStartClicked(bool)));
}


void MainWindow::CreateSliders()
{
	slider_layer_ = new QSlider(Qt::Horizontal);
	slider_layer_->setMinimum(0);
	slider_layer_->setMaximum(10);
	connect(slider_layer_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintLayer(int)));

	slider_order_ = new QSlider(Qt::Horizontal);
	slider_order_->setMinimum(0);
	slider_order_->setMaximum(20);
	connect(slider_order_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(PrintOrder(int)));

	slider_scale_ = new QSlider(Qt::Horizontal);
	slider_scale_->setMinimum(1);
	slider_scale_->setMaximum(20);
	slider_scale_->setValue(10);
	connect(slider_scale_, SIGNAL(valueChanged(int)), renderingwidget_, SLOT(ScaleFrame(int)));
	connect(slider_scale_, SIGNAL(valueChanged(int)), this, SLOT(ShowScale(int)));
}


void MainWindow::CreateGroups()
{
	// render group
	connect(this, SIGNAL(EdgeMode(int)), renderingwidget_, SLOT(CheckEdgeMode(int)));

	groupbox_render_ = new QGroupBox(tr("Render"), this);
	groupbox_render_->setFlat(true);

	QVBoxLayout* render_layout = new QVBoxLayout(groupbox_render_);
	render_layout->addWidget(checkbox_point_);
	render_layout->addWidget(checkbox_light_);
	render_layout->addWidget(checkbox_axes_);

	// edge group
	groupbox_edge_ = new QGroupBox(tr("Edge"), this);
	groupbox_edge_->setCheckable(true);
	connect(groupbox_edge_, SIGNAL(clicked(bool)), this, SLOT(EdgeModeChange()));

	QVBoxLayout* edge_layout = new QVBoxLayout(groupbox_edge_);
	edge_layout->addWidget(radiobutton_heat_);
	edge_layout->addWidget(radiobutton_cut_);
	//edge_layout->addWidget(slider_layer_);
	edge_layout->addWidget(radiobutton_bulk_);
	edge_layout->addWidget(radiobutton_order_);

	edge_layout->addWidget(radiobutton_none_);

	// order display group
	groupbox_orderdisplay_ = new QGroupBox(tr("Display"), this);
	groupbox_orderdisplay_->setFlat(true);

	QVBoxLayout* orderdisplay_layout = new QVBoxLayout(groupbox_orderdisplay_);
	orderdisplay_layout->addWidget(slider_order_);
	
	// order settings group
	groupbox_ordersettings_ = new QGroupBox(tr("Settings"), this);
	groupbox_ordersettings_->setFlat(true);

	QVBoxLayout* ordersettings_layout = new QVBoxLayout(groupbox_ordersettings_);
	ordersettings_layout->addWidget(pushbutton_orientation_);
	ordersettings_layout->addWidget(toolbutton_setstart_);

	// edit group
	groupbox_edit_ = new QGroupBox(tr("Edit"), this);
	groupbox_edit_->setFlat(true);

	QVBoxLayout* edit_layout = new QVBoxLayout(groupbox_edit_);
	edit_layout->addWidget(pushbutton_rotatexy_);
	edit_layout->addWidget(pushbutton_rotatexz_);
	edit_layout->addWidget(pushbutton_rotateyz_);
	edit_layout->addWidget(toolbutton_addedge_);
	edit_layout->addWidget(pushbutton_simplify_);

	// scale group
	groupbox_scale_ = new QGroupBox(tr("Scale"), this);
	groupbox_scale_->setFlat(true);

	QVBoxLayout* scale_layout = new QVBoxLayout(groupbox_scale_);
	scale_layout->addWidget(slider_scale_);

	// separator group
	groupbox_separator_ = new QGroupBox(this);
	groupbox_separator_->setFlat(true);

	// fiber group
	groupbox_fiber_ = new QGroupBox(tr("Fiber"), this);

	QVBoxLayout* fiber_layout = new QVBoxLayout(groupbox_fiber_);
	fiber_layout->addWidget(pushbutton_fiberprint_);
	fiber_layout->addWidget(toolbutton_choosebound_);
	fiber_layout->addWidget(pushbutton_project_);

	// parameter group
	groupbox_para_ = new QGroupBox(tr("Parameter"), this);

	QVBoxLayout* para_layout = new QVBoxLayout(groupbox_para_);
	para_layout->addWidget(label_radius_);
	para_layout->addWidget(line_radius_);
	para_layout->addWidget(label_density_);
	para_layout->addWidget(line_density_);
	para_layout->addWidget(label_g_);
	para_layout->addWidget(line_g_);
	para_layout->addWidget(label_youngsmodulus_);
	para_layout->addWidget(line_youngsmodulus_);
	para_layout->addWidget(label_shearmodulus_);
	para_layout->addWidget(line_shearmodulus_);

	para_layout->addWidget(label_penalty_);
	para_layout->addWidget(line_penalty_);
	para_layout->addWidget(label_Dtol_);
	para_layout->addWidget(line_Dtol_);
	para_layout->addWidget(label_pritol_);
	para_layout->addWidget(line_pritol_);
	para_layout->addWidget(label_dualtol_);
	para_layout->addWidget(line_dualtol_);

	para_layout->addWidget(label_alpha_);
	para_layout->addWidget(line_alpha_);
	para_layout->addWidget(label_beta_);
	para_layout->addWidget(line_beta_);
	para_layout->addWidget(label_gamma_);
	para_layout->addWidget(line_gamma_);
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


void MainWindow::AddEdgeClicked(bool down)
{
	toolbutton_addedge_->setDown(down);
}


void MainWindow::ChooseBoundClicked(bool down)
{
	toolbutton_choosebound_->setDown(down);
}


void MainWindow::SetStartClicked(bool down)
{
	toolbutton_setstart_->setDown(down);
}


void MainWindow::GetParameters()
{
	emit(SendParameters(
		line_radius_->text().toDouble(),
		line_density_->text().toDouble(),
		line_g_->text().toDouble(),
		line_youngsmodulus_->text().toDouble(),
		line_shearmodulus_->text().toDouble(),
		line_penalty_->text().toDouble(),
		line_Dtol_->text().toDouble(),
		line_pritol_->text().toDouble(),
		line_dualtol_->text().toDouble(),
		line_alpha_->text().toDouble(),
		line_beta_->text().toDouble(),
		line_gamma_->text().toDouble()
		)
	);
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


void MainWindow::ShowScale(int size)
{
	double scale = size*1.0 / 10;
	label_operatorinfo_->setText(QString("Scale: %1").arg(scale*scale));
}


void MainWindow::EdgeModeChange()
{
	if (groupbox_edge_->isChecked())
	{
		if (sender() == radiobutton_heat_)
		{
			if (edge_render_ != HEAT)
			{
				radiobutton_heat_->setChecked(true);
				edge_render_ = HEAT;
				emit(EdgeMode(HEAT));

				groupbox_orderdisplay_->setVisible(false);
				groupbox_ordersettings_->setVisible(false);
				groupbox_edit_->setVisible(true);
				groupbox_scale_->setVisible(true);

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

				groupbox_orderdisplay_->setVisible(false);
				groupbox_ordersettings_->setVisible(false);
				groupbox_edit_->setVisible(true);
				groupbox_scale_->setVisible(true);

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

				groupbox_orderdisplay_->setVisible(false);
				groupbox_ordersettings_->setVisible(false);
				groupbox_edit_->setVisible(true);
				groupbox_scale_->setVisible(true);

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

				groupbox_edit_->setVisible(false);
				groupbox_scale_->setVisible(false);
				groupbox_orderdisplay_->setVisible(true);
				groupbox_ordersettings_->setVisible(true);

				return;
			}
		}

		radiobutton_none_->setChecked(true);
		edge_render_ = EDGE;
		emit(EdgeMode(EDGE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_ordersettings_->setVisible(false);
		groupbox_edit_->setVisible(true);
		groupbox_scale_->setVisible(true);
	}
	else
	{
		radiobutton_none_->setChecked(true);
		edge_render_ = NONE;
		emit(EdgeMode(NONE));

		groupbox_orderdisplay_->setVisible(false);
		groupbox_ordersettings_->setVisible(false);
		groupbox_edit_->setVisible(true);
		groupbox_scale_->setVisible(true);
	}
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


void MainWindow::Reset()
{
	slider_layer_->setValue(0);
	slider_order_->setValue(0);
	slider_scale_->setValue(10);
	label_operatorinfo_->setText(QString("Scale: 1.0"));

	groupbox_edge_->setChecked(false);
	radiobutton_none_->setChecked(true);
	edge_render_ = NONE;

	groupbox_orderdisplay_->setVisible(false);
	groupbox_ordersettings_->setVisible(false);
	groupbox_edit_->setVisible(true);
	groupbox_scale_->setVisible(true);
}
/*
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
*/