﻿#include "renderingwidget.h"


RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
: QGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(10.0),
has_lighting_(false), is_draw_point_(true), is_draw_edge_(false), is_draw_heat_(false),
is_draw_bulk_(false), is_draw_axes_(false), op_mode_(NORMAL), scale_(1.0)
{
	ptr_arcball_ = new CArcBall(width(), height());
	ptr_frame_ = NULL;
	ptr_fiberprint_ = NULL;

	eye_goal_[0] = eye_goal_[1] = eye_goal_[2] = 0.0;
	eye_direction_[0] = eye_direction_[1] = 0.0;
	eye_direction_[2] = 1.0;

	setFocusPolicy(Qt::StrongFocus);
}


RenderingWidget::~RenderingWidget()
{
	SafeDelete(ptr_arcball_);
	SafeDelete(ptr_frame_);
	SafeDelete(ptr_fiberprint_);
}


void RenderingWidget::InitDrawData()
{
	is_draw_edge_ = false;
	is_draw_heat_ = false;
	is_draw_bulk_ = false;
	is_draw_order_ = false;
}


void RenderingWidget::InitCapturedData()
{
	op_mode_ = NORMAL;

	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	captured_verts_.clear();
	is_captured_vert_.resize(N);
	fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
	captured_edges_.clear();
	is_captured_edge_.resize(M);
	fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);

	emit(CapturedVert(-1, -1));
}


void RenderingWidget::InitFiberData()
{
	print_order_ = 0;
}


void RenderingWidget::initializeGL()
{
	glClearColor(0.3, 0.3, 0.3, 0.0);
	glShadeModel(GL_SMOOTH);

	glEnable(GL_DOUBLEBUFFER);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_DEPTH_TEST);
	glClearDepth(1);

	SetLight();

}


void RenderingWidget::resizeGL(int w, int h)
{
	h = (h == 0) ? 1 : h;

	ptr_arcball_->reSetBound(w, h);

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0, GLdouble(w) / GLdouble(h), 0.001, 1000);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void RenderingWidget::paintGL()
{
	glShadeModel(GL_SMOOTH);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//add guoxian
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//

	if (has_lighting_)
	{
		SetLight();
	}
	else
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_LIGHT0);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	register vec eyepos = eye_distance_*eye_direction_;
	gluLookAt(eyepos[0], eyepos[1], eyepos[2],
		eye_goal_[0], eye_goal_[1], eye_goal_[2],
		0.0, 1.0, 0.0);
	glPushMatrix();

	glMultMatrixf(ptr_arcball_->GetBallMatrix());

	Render();
	glPopMatrix();

}

void RenderingWidget::timerEvent(QTimerEvent * e)
{
	updateGL();
}


void RenderingWidget::mousePressEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		ptr_arcball_->MouseDown(e->pos());
		if (op_mode_ == CHOOSEBASE)
		{
			CaptureVertex(e->pos());
		}
		else
		if (op_mode_ == CHOOSECEILING)
		{
			CaptureEdge(e->pos());
		}
		else
		{
			if (!CaptureVertex(e->pos()))
			{
				CaptureEdge(e->pos());
			}
		}
		break;
	case Qt::MidButton:
		current_position_ = e->pos();
		break;
	case Qt::RightButton:
		if (op_mode_ == CHOOSEBASE)
		{
			if (captured_verts_.size() > 0)
			{
				WF_vert *u = captured_verts_[captured_verts_.size() - 1];
				captured_verts_.pop_back();
				is_captured_vert_[u->ID()] = false;
			}
		}
		else
		if (op_mode_ == CHOOSECEILING)
		{
			if (captured_edges_.size() > 0)
			{
				WF_edge *e = captured_edges_[captured_edges_.size() - 1];
				captured_edges_.pop_back();
				is_captured_edge_[e->ID()] = false;
				is_captured_edge_[e->ppair_->ID()] = false;
			}
		}
		else
		{
		}
		break;
	default:
		break;
	}

	updateGL();
}


void RenderingWidget::mouseMoveEvent(QMouseEvent *e)
{
	switch (e->buttons())
	{
		setCursor(Qt::ClosedHandCursor);
	case Qt::LeftButton:
		ptr_arcball_->MouseMove(e->pos());
		break;
	case Qt::MidButton:
		eye_goal_[0] -= 4.0*GLfloat(e->x() - current_position_.x()) / GLfloat(width());
		eye_goal_[1] += 4.0*GLfloat(e->y() - current_position_.y()) / GLfloat(height());
		current_position_ = e->pos();
		break;
	default:
		break;
	}

	updateGL();
}


void RenderingWidget::mouseDoubleClickEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		break;
	default:
		break;
	}
	updateGL();
}


void RenderingWidget::mouseReleaseEvent(QMouseEvent *e)
{
	switch (e->button())
	{
	case Qt::LeftButton:
		ptr_arcball_->MouseUp(e->pos());
		setCursor(Qt::ArrowCursor);
		break;

	case Qt::RightButton:
		break;
	default:
		break;
	}
}


void RenderingWidget::wheelEvent(QWheelEvent *e)
{
	eye_distance_ += e->delta()*0.005;
	eye_distance_ = eye_distance_ < 0 ? 0 : eye_distance_;

	updateGL();
}


void RenderingWidget::keyPressEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_C:
		//SwitchToChooseBound();
		break;
	/*
	case Qt::Key_I:
		SwitchToAddEdge();
		break;
	case Qt::Key_F:
		SwitchToAddFace();
		break;
	*/
	case Qt::Key_Escape:
		SwitchToNormal();
		break;
	default:
		break;
	}
	updateGL();
}


void RenderingWidget::keyReleaseEvent(QKeyEvent *e)
{
	switch (e->key())
	{
	case Qt::Key_A:
		break;
	default:
		break;
	}
}


void RenderingWidget::CoordinatesTransform(QPoint p, double *objx, double *objy, double *objz)
{
	double modelview[16];
	double projection[16];
	int viewport[4];

	glPushMatrix();

	glMultMatrixf(ptr_arcball_->GetBallMatrix());

	// Read the projection, modelview and viewport matrices using the glGet functions.
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetIntegerv(GL_VIEWPORT, viewport);

	glPopMatrix();

	// Read the window z value from the z-buffer 
	double winx = p.rx();
	double winy = viewport[3] - p.ry();
	float winz = 1.0;
	glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);

	// Use the gluUnProject to get the world co-ordinates of the point the user clicked and save in objx, objy, objz.
	gluUnProject(winx, winy, winz, modelview, projection, viewport, objx, objy, objz);
}


bool RenderingWidget::CaptureVertex(QPoint mouse)
{
	if (ptr_frame_ == NULL)
	{
		return 0;
	}

	double x = 0;
	double y = 0;
	double z = 0;
	CoordinatesTransform(mouse, &x, &y, &z);

	vector<WF_vert*> verts = *(ptr_frame_->GetVertList());
	int N = verts.size();
	for (int i = 0; i < N; i++)
	{
		double dx = verts[i]->RenderPos().x() - x;
		double dy = verts[i]->RenderPos().y() - y;
		double dz = verts[i]->RenderPos().z() - z;
		double dis = sqrt(dx*dx + dy*dy + dz*dz);
		if (dis < 0.015)
		{
			if (op_mode_ == NORMAL)
			{
				captured_verts_.clear();
				fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
				captured_edges_.clear();
				fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);
			}

			if (!is_captured_vert_[i])
			{
				captured_verts_.push_back(verts[i]);
				is_captured_vert_[i] = true;
				emit(CapturedVert(i + 1, verts[i]->Degree()));
			}

			return true;
		}
	}

	return false;
}


bool RenderingWidget::CaptureEdge(QPoint mouse)
{
	if (ptr_frame_ == NULL)
	{
		return 0;
	}

	double x = 0;
	double y = 0;
	double z = 0;
	CoordinatesTransform(mouse, &x, &y, &z);

	vector<WF_edge*> edges = *(ptr_frame_->GetEdgeList());
	int M = edges.size();
	for (size_t i = 0; i < M; i++)
	{
		if (edges[i]->ID() < edges[i]->ppair_->ID())
		{
			WF_vert	u = WF_vert(x, y, z);
			WF_vert *v1 = edges[i]->pvert_;
			WF_vert *v2 = edges[i]->ppair_->pvert_;

			double delta = ptr_frame_->ArcHeight(u.RenderPos(), v1->RenderPos(), v2->RenderPos());
			if (delta < 0.007)
			{
				if (op_mode_ == NORMAL)
				{
					captured_verts_.clear();
					fill(is_captured_vert_.begin(), is_captured_vert_.end(), false);
					captured_edges_.clear();
					fill(is_captured_edge_.begin(), is_captured_edge_.end(), false);
				}
	
				if (!is_captured_edge_[i])
				{
					captured_edges_.push_back(edges[i]);
					is_captured_edge_[i] = true;
					is_captured_edge_[edges[i]->ppair_->ID()] = true;
					emit(CapturedEdge(i + 1, edges[i]->Length()));
				}

				return true;
			}
		}
	}

	return false;
}


void RenderingWidget::Render()
{
	DrawAxes(is_draw_axes_);
	DrawPoints(is_draw_point_);
	DrawEdge(is_draw_edge_);
	DrawHeat(is_draw_heat_);
	DrawBulk(is_draw_bulk_);
	DrawOrder(is_draw_order_);
}


void RenderingWidget::SetLight()
{
	static GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat mat_shininess[] = { 50.0 };
	static GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };
	static GLfloat white_light[] = { 0.8, 0.8, 0.8, 1.0 };
	static GLfloat lmodel_ambient[] = { 0.3, 0.3, 0.3, 1.0 };

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
}


void RenderingWidget::SetBackground()
{
	QColor color = QColorDialog::getColor(Qt::white, this, tr("background color"));
	GLfloat r = (color.red()) / 255.0f;
	GLfloat g = (color.green()) / 255.0f;
	GLfloat b = (color.blue()) / 255.0f;
	GLfloat alpha = color.alpha() / 255.0f;
	glClearColor(r, g, b, alpha);
	updateGL();
}


void RenderingWidget::ScaleFrame(double scale)
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();
	float size = scale / scale_;
	for (int i = 0; i < N; i++)
	{
		Vec3f p = verts[i]->Position();
		verts[i]->SetPosition(p * size);
	}
	scale_ = scale;
	ptr_frame_->Unify();

	int cape_size = captured_edges_.size();
	if (cape_size != 0)
	{
		emit(CapturedEdge(captured_edges_[cape_size - 1]->ID() + 1,
			captured_edges_[cape_size - 1]->Length()));
	}

	updateGL();
}


void RenderingWidget::ReadFrame()
{
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
		"..", tr("Mesh files(*.obj *.pwf)"));

	if (filename.isEmpty())
	{
		emit(operatorInfo(QString("Read Mesh Failed!")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	delete ptr_frame_; 
	ptr_frame_ = new WireFrame();

	if (filename.contains(".obj") || filename.contains(".OBJ"))
	{
		ptr_frame_->LoadFromOBJ(byfilename.data());
	}
	else
	{
		ptr_frame_->LoadFromPWF(byfilename.data());
	}

	emit(ChooseBasePressed(false));
	emit(ChooseCeilingPressed(false));

	emit(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	// emit(operatorInfo(QString("Read Mesh from") + filename + QString(" Done")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));
	emit(Reset());

	InitDrawData();
	InitCapturedData();
	InitFiberData();

	updateGL();
}


void RenderingWidget::WriteFrame(QString filename)
{
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		emit(QString("The mesh is Empty !"));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	ptr_frame_->WriteToOBJ(filename.toLatin1().data());

	emit(operatorInfo(QString("Write mesh to ") + filename + QString(" Done")));
}


void RenderingWidget::WriteFrame(bool bVert, bool bLine, 
	bool bBase, bool bCeiling, bool bCut, 
	int min_layer, int max_layer, QString filename)
{
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		emit(QString("The mesh is Empty !"));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	ptr_frame_->WriteToPWF(bVert, bLine, bBase, bCeiling, bCut,
		min_layer, max_layer, filename.toLatin1().data());

	emit(operatorInfo(QString("Write mesh to ") + filename + QString(" Done")));
}


void RenderingWidget::ImportFrame()
{
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Import Mesh"),
		"..");

	if (filename.isEmpty())
	{
		emit(operatorInfo(QString("Import Mesh Failed!")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray byfilename = filename.toLocal8Bit();

	delete ptr_frame_;
	ptr_frame_ = new WireFrame();

	ptr_frame_->ImportFrom3DD(byfilename.data());

	emit(ChooseBasePressed(false));
	emit(ChooseCeilingPressed(false));

	emit(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	// emit(operatorInfo(QString("Read Mesh from") + filename + QString(" Done")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));
	emit(Reset());

	InitDrawData();
	InitCapturedData();
	InitFiberData();

	updateGL();
}


void RenderingWidget::ExportFrame(int min_layer, int max_layer,
	QString vert_path, QString line_path)
{	
	if (ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		emit(QString("The mesh is Empty !"));
		return;
	}

	if (min_layer == 0 || max_layer == 0 || min_layer > max_layer)
	{
		emit(Error(QString("Invalid layer information")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);

	if (!vert_path.isEmpty())
	{
		ptr_frame_->ExportPoints(min_layer, max_layer,
			vert_path.toLocal8Bit().data());
	}

	if (!line_path.isEmpty())
	{
		ptr_frame_->ExportLines(min_layer, max_layer,
			line_path.toLocal8Bit().data());
	}

	emit(operatorInfo(QString("Export mesh done")));	
}


void RenderingWidget::CheckDrawPoint(bool bv)
{
	is_draw_point_ = bv;
	updateGL();
}


void RenderingWidget::CheckEdgeMode(int type)
{
	switch (type)
	{
	case NONE:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case EDGE:
		is_draw_edge_ = true;
		is_draw_heat_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case HEAT:
		is_draw_edge_ = false;
		is_draw_heat_ = true;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case BULK:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_bulk_ = true;
		is_draw_order_ = false;
		break;

	case ORDER:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = true;
		break;

	default:
		break;
	}
	updateGL();
}


void RenderingWidget::CheckLight(bool bv)
{
	has_lighting_ = bv;
	updateGL();
}


void RenderingWidget::CheckDrawAxes(bool bV)
{
	is_draw_axes_ = bV;
	updateGL();
}


void RenderingWidget::SwitchToNormal()
{
	emit(ChooseBasePressed(false));
	emit(ChooseCeilingPressed(false));

	if (ptr_frame_ == NULL)
	{
		emit(modeInfo(QString("")));
	}
	else
	{
		emit(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	}

	if (op_mode_ == CHOOSEBASE)
	{
		base_.clear();
		for (int i = 0; i < captured_verts_.size(); i++)
		{
			base_.push_back(captured_verts_[i]);
		}
	}
	else
	if (op_mode_ == CHOOSECEILING)
	{
		ceiling_.clear();
		for (int i = 0; i < captured_edges_.size(); i++)
		{
			ceiling_.push_back(captured_edges_[i]);
		}
		
		ptr_frame_->MakeCeiling(ceiling_);
	}

	InitCapturedData();
}


void RenderingWidget::SwitchToChooseBase()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSEBASE)
	{
		SwitchToNormal();
	}
	else
	{
		emit(ChooseCeilingPressed(false));
		emit(CapturedVert(-1, -1));

		emit(ChooseBasePressed(true));
		emit(modeInfo(QString("Choosing base...Press again or press ESC to exit.")));
		op_mode_ = CHOOSEBASE;
	}
}


void RenderingWidget::SwitchToChooseCeiling()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSECEILING)
	{
		SwitchToNormal();
	}
	else
	{
		emit(ChooseBasePressed(false));
		emit(CapturedVert(-1, -1));

		emit(ChooseCeilingPressed(true));
		emit(modeInfo(QString("Choosing ceiling...Press again or press ESC to exit.")));
		op_mode_ = CHOOSECEILING;
	}
}


void RenderingWidget::DrawAxes(bool bV)
{
	if (!bV)
		return;
	//x axis
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.7, 0.0, 0.0);
	glEnd();
	glPushMatrix();
	glTranslatef(0.7, 0, 0);
	glRotatef(90, 0.0, 1.0, 0.0);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//y axis
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.7, 0.0);
	glEnd();

	glPushMatrix();
	glTranslatef(0.0, 0.7, 0);
	glRotatef(90, -1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	//z axis
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.0, 0.0, 0.7);
	glEnd();
	glPushMatrix();
	glTranslatef(0.0, 0, 0.7);
	glutSolidCone(0.02, 0.06, 20, 10);
	glPopMatrix();

	glColor3f(1.0, 1.0, 1.0);
}


void RenderingWidget::DrawPoints(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_frame_->SizeOfVertList() == 0)
	{
		return;
	}

	const std::vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	glPointSize(5.0f);
	glBegin(GL_POINTS);

	for (size_t i = 0; i <N; i++)
	{
		glColor3f(1.0, 1.0, 1.0);

		if (verts[i]->isFixed())
		{
			glColor3f(0.0, 1.0, 1.0);
		}

		if (is_captured_vert_[i])
		{
			glColor3f(1.0, 0.0, 0.0);
		}

		glVertex3fv(verts[i]->RenderPos().data());
	}

	glColor3f(1.0, 1.0, 1.0);
	glEnd();
}


void RenderingWidget::DrawEdge(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_frame_->SizeOfEdgeList() == 0)
	{
		return;
	}

	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	for (size_t i = 0; i < M; i++)
	{
		WF_edge *e = edges[i];
		WF_edge *e_pair = edges[i]->ppair_;

		if (e->ID() < e_pair->ID())
		{
			glBegin(GL_LINE_LOOP);

			glColor3f(1.0, 1.0, 1.0);

			if (e->isPillar())
			{
				glColor3f(0.0, 1.0, 1.0);
			}

			if (e->isCeiling())
			{
				glColor3f(1.0, 1.0, 0.0);
			}

			if (is_captured_edge_[i])
			{
				glColor3f(1.0, 0.0, 0.0);
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}
}


void RenderingWidget::DrawHeat(bool bv)
{
	if (!bv || ptr_frame_ == NULL)
	{
		return;
	}

	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			WF_edge *e = edges[i];
			WF_edge *e_pair = edges[i]->ppair_;

			glBegin(GL_LINE_LOOP);

			int tag = (e->Layer() % 6);
			switch (tag)
			{
			case 0:
				glColor3f(1.0, 0.0, 0.0);
				break;
			case 1:
				glColor3f(0.0, 1.0, 0.0);
				break;
			case 2:
				glColor3f(0.0, 0.0, 1.0);
				break;
			case 3:
				glColor3f(1.0, 1.0, 0.0);
				break;
			case 4:
				glColor3f(1.0, 0.0, 1.0);
				break;
			case 5:
				glColor3f(1.0, 1.0, 1.0);
				break;

			default:
				break;
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}
}


void RenderingWidget::DrawBulk(bool bv)
{
	//if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == NULL)
	//{
	//	return;
	//}

	//const vector<DualVertex*> dual_vert = *(ptr_fiberprint_->GetDualVertList());
	//const vector<BaseBulk*> bulk_list = *(ptr_fiberprint_->GetBulk());
	//vector<vector<int>> range_state = *(ptr_fiberprint_->GetRangeState());
	//const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	//int M = ptr_frame_->SizeOfEdgeList();

	///*Draw Collision*/
	//for (size_t i = 0; i < M; i++)
	//{
	//	WF_edge *e = edges[i];
	//	WF_edge *e_pair = edges[i]->ppair_;

	//	if (e->ID() < e_pair->ID())
	//	{
	//		glBegin(GL_LINE_LOOP);

	//		//decide line color
	//		if (captured_edge_ != NULL)
	//		{
	//			int e_id = dual_vert[i]->dual_id();
	//			int cap_id = dual_vert[captured_edge_->ID()]->dual_id();
	//			if (captured_edge_->ID() == i)
	//			{
	//				glColor4f(0.0, 0.0, 1.0, 1);

	//				// draw bulk
	//				if (!has_lighting_)
	//				{
	//					glDepthMask(GL_FALSE);
	//				}
	//				
	//					BaseBulk *bulk = bulk_list[cap_id];
	//				if (bulk)
	//				{
	//					if (bulk->flag == 1)
	//					{
	//					bulk->Face(0)->Render(ptr_frame_, 0.1);
	//					bulk->Face(1)->Render(ptr_frame_, 0.6);
	//					bulk->Face(2)->Render(ptr_frame_, 0.2);
	//					bulk->Face(3)->Render(ptr_frame_, 0.2);
	//					bulk->Face(4)->Render(ptr_frame_, 0.35);
	//					bulk->Face(5)->Render(ptr_frame_, 0.35);
	//					bulk->Face(6)->Render(ptr_frame_, 0.4);
	//					bulk->Face(7)->Render(ptr_frame_, 0.4);
	//					bulk->Face(8)->Render(ptr_frame_, 0.3);
	//					bulk->Face(9)->Render(ptr_frame_, 0.3);
	//					bulk->Face(10)->Render(ptr_frame_, 0.3);
	//					bulk->Face(11)->Render(ptr_frame_, 0.5);
	//					bulk->Face(12)->Render(ptr_frame_, 0.3);
	//					bulk->Face(13)->Render(ptr_frame_, 0.3);

	//					}
	//					if (bulk->flag == 2)
	//					{
	//					
	//							bulk->Face(0)->Render(ptr_frame_, 0.1);
	//							bulk->Face(1)->Render(ptr_frame_, 0.1);
	//							bulk->Face(2)->Render(ptr_frame_, 0.1);
	//							bulk->Face(3)->Render(ptr_frame_, 0.1);
	//							bulk->Face(4)->Render(ptr_frame_, 0.1);
	//							bulk->Face(5)->Render(ptr_frame_, 0.1);
	//							bulk->Face(6)->Render(ptr_frame_, 0.5);
	//							bulk->Face(7)->Render(ptr_frame_, 0.5);
	//							bulk->Face(8)->Render(ptr_frame_, 0.3);
	//							bulk->Face(9)->Render(ptr_frame_, 0.3);
	//							bulk->Face(10)->Render(ptr_frame_, 0.3);
	//							bulk->Face(11)->Render(ptr_frame_, 0.5);
	//							bulk->Face(12)->Render(ptr_frame_, 0.4);		
	//							bulk->Face(13)->Render(ptr_frame_, 0.4);
	//							bulk->Face(14)->Render(ptr_frame_, 0.4);
	//					}
	//					
	//				}
	//				if (!has_lighting_)
	//				{
	//					glDepthMask(GL_TRUE);
	//				}
	//			}
	//			else
	//			if (range_state[cap_id][e_id] == 1)
	//			{
	//				glColor4f(0.00, 1, 0.00, 0.4);
	//			}
	//			else
	//			if (range_state[cap_id][e_id] == 2)
	//			{
	//				glColor4f(1.0, 0.0, 0.0, 1);
	//			}
	//			else
	//			{
	//				glColor4f(1.0, 1.0, 1.0, 1);
	//			}
	//		}
	//		else
	//		{
	//			glColor4f(1.0, 1.0, 1.0, 1);
	//		}

	//		glVertex3fv(e->pvert_->RenderPos().data());
	//		glVertex3fv(e->ppair_->pvert_->RenderPos().data());

	//		glEnd();
	//	}
	//}
}


void RenderingWidget::DrawOrder(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == NULL)
	{
		return;
	}

	if (op_mode_ == NORMAL)
	{
		vector<int> print_queue; 
		ptr_fiberprint_->GetQueue(print_queue);
		for (int i = 0; i < print_order_; i++)
		{
			WF_edge *e = ptr_frame_->GetEdge(print_queue[i]);
			glBegin(GL_LINE_LOOP);
			glColor3f(1.0, 1.0, 1.0);
			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());
			glEnd();
		}

		if (print_order_ > 0)
		{
			//ptr_fiberprint_->ptr_seqanalyzer_->GetExtru(print_order_ - 1).Render(ptr_frame_, 0.5);
		}
	}

}


void RenderingWidget::FiberPrintAnalysis(double radius, double density, double g,
										double youngs_modulus, double shear_modulus,
										double Dt_tol, double Dr_tol,
										double penalty, double pri_tol, double dual_tol,
										double gamma, double Wl, double Wp)
{
	QString dirname = QFileDialog::
		getExistingDirectory(this, 
							tr("Result Directory"),
							"/home",
							QFileDialog::ShowDirsOnly
							| QFileDialog::DontResolveSymlinks);

	if (dirname.isEmpty())
	{
		emit(operatorInfo(QString("Read Directory Failed!")));
		return;
	}

	// compatible with paths in chinese
	QTextCodec *code = QTextCodec::codecForName("gd18030");
	QTextCodec::setCodecForLocale(code);
	QByteArray bydirname = dirname.toLocal8Bit();

	
	FiberPrintPARM *ptr_parm = new FiberPrintPARM(
		radius, density, g, 
		youngs_modulus, shear_modulus, 
		Dt_tol, Dr_tol, 
		penalty, pri_tol, dual_tol, 
		gamma, Wl, Wp);

	delete ptr_fiberprint_; 
	ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm, bydirname.data());
	ptr_fiberprint_->Print();

	emit(SetOrderSlider(0));
	emit(SetMaxOrderSlider(ptr_fiberprint_->ptr_graphcut_->ptr_dualgraph_->SizeOfVertList()));

	delete ptr_parm;
}


void RenderingWidget::SimplifyFrame()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}
	ptr_frame_->SimplifyFrame();

	emit(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	emit(operatorInfo(QString("")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

	updateGL();
}


void RenderingWidget::RefineFrame()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}
	ptr_frame_->RefineFrame();

	emit(operatorInfo(QString("")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

	updateGL();
}

void RenderingWidget::DebugFrame()
{
	QuadricCollision* test = new QuadricCollision();
	test->Debug();
}


void RenderingWidget::ProjectBound(double len)
{
	if (base_.size() <= 0)
	{
		return;
	}
	ptr_frame_->ProjectBound(base_, len);

	emit(modeInfo(QString("Choose base (B) | Choose ceiling (C)")));
	emit(operatorInfo(QString("")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

	InitCapturedData();
	updateGL();
}


void RenderingWidget::ModifyProjection(double len)
{
	ptr_frame_->ModifyProjection(len);
	updateGL();
}


void RenderingWidget::RotateXY()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().y(), verts[i]->Position().x(),
			verts[i]->Position().z());
		verts[i]->SetRenderPos(verts[i]->RenderPos().y(), verts[i]->RenderPos().x(),
			verts[i]->RenderPos().z());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::RotateXZ()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().z(), verts[i]->Position().y(),
			verts[i]->Position().x());
		verts[i]->SetRenderPos(verts[i]->RenderPos().z(), verts[i]->RenderPos().y(),
			verts[i]->RenderPos().x());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::RotateYZ()
{
	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();

	for (int i = 0; i < N; i++)
	{
		verts[i]->SetPosition(verts[i]->Position().x(), verts[i]->Position().z(),
			verts[i]->Position().y());
		verts[i]->SetRenderPos(verts[i]->RenderPos().x(), verts[i]->RenderPos().z(),
			verts[i]->RenderPos().y());
	}

	ptr_frame_->Unify();
	updateGL();
}


void RenderingWidget::PrintOrder(int order)
{
	print_order_ = order;
	updateGL();
}


void RenderingWidget::PrintNextStep()
{
	int M = ptr_frame_->SizeOfEdgeList() / 2;
	if (print_order_ < M - 1)
	{
		print_order_++;
	}

	updateGL();

	emit(SetOrderSlider(print_order_));
}


void RenderingWidget::PrintNextLayer()
{
	vector<int> print_queue;
	ptr_fiberprint_->GetQueue(print_queue);

	int next_layer;
	if (print_order_ == 0)
	{
		next_layer = 1;
	}
	else
	{
		int orig_e = print_queue[print_order_ - 1];
		next_layer = ptr_frame_->GetEdge(orig_e)->Layer() + 2; 
	}


	int M = ptr_frame_->SizeOfEdgeList() / 2;
	while (1)
	{
		if (print_order_ == M)
		{
			break;
		}

		int orig_e = print_queue[print_order_];
		if (ptr_frame_->GetEdge(orig_e)->Layer() == next_layer)
		{
			break;
		}
		print_order_++;
	}

	updateGL();

	emit(SetOrderSlider(print_order_));
}