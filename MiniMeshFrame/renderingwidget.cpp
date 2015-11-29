#include "renderingwidget.h"



RenderingWidget::RenderingWidget(QWidget *parent, MainWindow* mainwindow)
: QGLWidget(parent), ptr_mainwindow_(mainwindow), eye_distance_(10.0),
has_lighting_(false), is_draw_point_(true), is_draw_edge_(false), is_draw_heat_(false), is_draw_cut_(false), 
is_draw_bulk_(false), is_draw_axes_(false), is_simplified_(false), op_mode_(NORMAL), scale_(1.0)
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
	is_draw_cut_ = false;
	is_draw_bulk_ = false;
	is_draw_order_ = false;
}


void RenderingWidget::InitCapturedData()
{
	op_mode_ = NORMAL;
	captured_verts_.clear();
	captured_edge_ = NULL;
	capturing_face_.corner_points_.clear();
	capturing_face_.is_captured_vert_.resize(ptr_frame_->SizeOfVertList());
	fill(capturing_face_.is_captured_vert_.begin(), capturing_face_.is_captured_vert_.end(), false);
	capturing_face_.is_captured_edge_.resize(ptr_frame_->SizeOfEdgeList());
	fill(capturing_face_.is_captured_edge_.begin(), capturing_face_.is_captured_edge_.end(), false);
}


void RenderingWidget::InitFiberData()
{
	print_layer_ = 0;
	print_order_ = 0;

	bound_.resize(ptr_frame_->SizeOfVertList());
	fill(bound_.begin(), bound_.end(), 0);

	is_simplified_ = false;
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
		if (op_mode_ == CHOOSEBOUND || op_mode_ == ADDEDGE)
		{
			CaptureVertex(e->pos());
		}
		else
		if (op_mode_ == ADDFACE)
		{
			CaptureRing(e->pos());
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
		if (op_mode_ != NORMAL&& captured_verts_.size() > 0)
		{
			captured_verts_.pop_back();
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
		SwitchToChooseBound();
		break;
	case Qt::Key_I:
		SwitchToAddEdge();
		break;
	case Qt::Key_F:
		SwitchToAddFace();
		break;
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
	int i;
	for (i = 0; i < N; i++)
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
				captured_edge_ = NULL;
			}

			captured_verts_.push_back(verts[i]);
			break;
		}
	}

	if (i >= N)
	{
		return false;
	}

	switch (op_mode_)
	{
	case NORMAL:
		if (captured_verts_.size() >= 1)
		{
			emit(CapturedEdge(-1, -1));
			emit(CapturedVert(captured_verts_[0]->ID() + 1, captured_verts_[0]->Degree()));
		}
		break;

	case CHOOSEBOUND:
		break;

	case ADDEDGE:
		if (captured_verts_.size() >= 2)
		{
			ptr_frame_->InsertEdge(captured_verts_[0], captured_verts_[1]);
			captured_verts_.clear();
			emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));
		}
		break;

	case ADDFACE:
		break;

	default:
		break;
	}

	return true;
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
					captured_edge_ = NULL;
				}

				captured_edge_ = edges[i];
				emit(CapturedVert(-1, -1));
				emit(CapturedEdge(i + 1, captured_edge_->Length()));
				return true;
			}
		}
	}

	return false;
}


bool RenderingWidget::CaptureRing(QPoint mouse)
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
	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			WF_vert	o = WF_vert(x, y, z);
			WF_vert *u1 = edges[i]->pvert_;
			WF_vert *v1 = edges[i]->ppair_->pvert_;

			double delta = ptr_frame_->ArcHeight(o.RenderPos(), u1->RenderPos(), v1->RenderPos());
			if (delta < 0.007)
			{
				if (!capturing_face_.is_captured_edge_[i])
				{
					capturing_face_.is_captured_edge_[i] = true;
					capturing_face_.is_captured_edge_[j] = true;

					vector<WF_vert*> queue;
					int h = 0;
					int t = 2;
					queue.push_back(u1);
					queue.push_back(v1);
					while (h < t)
					{
						WF_vert *u = queue[h];
						capturing_face_.is_captured_vert_[u->ID()] = true;
						if (u->Degree() > 2)
						{
							capturing_face_.corner_points_.push_back(u);
						}
						else
						{
							WF_edge *e = u->pedge_;
							while (e != NULL)
							{
								WF_vert *v = e->pvert_;
								capturing_face_.is_captured_edge_[e->ID()] = true;
								capturing_face_.is_captured_edge_[e->ppair_->ID()] = true;
								if (!capturing_face_.is_captured_vert_[v->ID()])
								{
									queue.push_back(v);
									t++;
								}
								e = e->pnext_;
							}
						}

						h++;
					}
				}
				
				emit(CapturedVert(-1, -1));
				emit(CapturedEdge(-1, -1));
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
	DrawCut(is_draw_cut_);
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


void RenderingWidget::ReadFrame()
{
	QString filename = QFileDialog::
		getOpenFileName(this, tr("Read Mesh"),
		"..", tr("Meshes (*.obj)"));

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
	ptr_frame_->LoadFromOBJ(byfilename.data());

	emit(AddEdgePressed(false));
	emit(ChooseBoundPressed(false));

	emit(modeInfo(QString("Insert edge (I)")));
	// emit(operatorInfo(QString("Read Mesh from") + filename + QString(" Done")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));
	emit(Reset());

	InitDrawData();
	InitCapturedData();
	InitFiberData();

	updateGL();
}


void RenderingWidget::WriteFrame()
{
	if (ptr_frame_->SizeOfVertList() == 0)
	{
		emit(QString("The Mesh is Empty !"));
		return;
	}
	QString filename = QFileDialog::
		getSaveFileName(this, tr("Write Mesh"),
		"..", tr("Meshes (*.obj)"));

	if (filename.isEmpty())
		return;

	ptr_frame_->WriteToOBJ(filename.toLatin1().data());

	emit(operatorInfo(QString("Write Mesh to ") + filename + QString(" Done")));
}


void RenderingWidget::ScaleFrame(int size)
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	vector<WF_vert*>& verts = *(ptr_frame_->GetVertList());
	int N = ptr_frame_->SizeOfVertList();
	float pre_scale = scale_;
	scale_ = size *1.0 / 10;
	for (int i = 0; i < N; i++)
	{
		Vec3f p = verts[i]->Position();
		verts[i]->SetPosition(p/pre_scale*scale_);
	}
	ptr_frame_->Unify();

	if (captured_edge_ != NULL)
	{
		emit(CapturedEdge(captured_edge_->ID() + 1, captured_edge_->Length()));
	}

	updateGL();
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
		is_draw_cut_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case EDGE:
		is_draw_edge_ = true;
		is_draw_heat_ = false;
		is_draw_cut_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case HEAT:
		is_draw_edge_ = false;
		is_draw_heat_ = true;
		is_draw_cut_ = false;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case CUT:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_cut_ = true;
		is_draw_bulk_ = false;
		is_draw_order_ = false;
		break;

	case BULK:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_cut_ = false;
		is_draw_bulk_ = true;
		is_draw_order_ = false;
		break;

	case ORDER:
		is_draw_edge_ = false;
		is_draw_heat_ = false;
		is_draw_cut_ = false;
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
	emit(AddEdgePressed(false));
	emit(AddFacePressed(false));
	emit(ChooseBoundPressed(false));

	if (ptr_frame_ == NULL)
	{
		emit(modeInfo(QString("")));
	}
	else
	{
		if (ptr_fiberprint_ != NULL)
		{
			emit(modeInfo(QString("Choose boundary (C) | Insert edge (I) | Set face (F)")));
		}
		else
		if (is_simplified_)
		{
			emit(modeInfo(QString("Choose boundary (C) | Insert edge (I) | Set face (F)")));
		}
		else
		{
			emit(modeInfo(QString("Insert edge (I) | Set face (F)")));
		}
	}

	if (op_mode_ == CHOOSEBOUND)
	{
		fill(bound_.begin(), bound_.end(), 0);
		for (int i = 0; i < captured_verts_.size(); i++)
		{
			bound_[captured_verts_[i]->ID()] = true;
		}
	}
	else
	if (op_mode_ == ADDFACE)
	{
		vector<bool> visited(ptr_frame_->SizeOfVertList());
		fill(visited.begin(), visited.end(), false);

		vector<WF_vert*> queue;
		queue.push_back(capturing_face_.corner_points_[0]);
		int h = 0;
		int t = 1;
		while (h < t)
		{
			WF_vert *u = queue[h];
			visited[u->ID()] = true;

			WF_edge *e = u->pedge_;
			while (e != NULL)
			{
				if (!visited[e->pvert_->ID()] && capturing_face_.is_captured_edge_[e->ID()])
				{
					queue.push_back(e->pvert_);
					t++;
					break;
				}
				e = e->pnext_;
			}
			h++;
		}

		ptr_frame_->InsertFace(queue);
	}

	InitCapturedData();
}


void RenderingWidget::SwitchToChooseBound()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == CHOOSEBOUND)
	{
		SwitchToNormal();
	}
	else
	{
		if (is_simplified_)
		{
			emit(AddEdgePressed(false));
			emit(AddFacePressed(false));
			emit(ChooseBoundPressed(true));
			emit(modeInfo(QString("Choosing boundary...Press again or press ESC to exit.")));
			op_mode_ = CHOOSEBOUND;
		}
	}
}


void RenderingWidget::SwitchToAddEdge()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == ADDEDGE)
	{
		SwitchToNormal();
	}
	else
	{
		emit(AddEdgePressed(true));
		emit(AddFacePressed(false));
		emit(ChooseBoundPressed(false));
		emit(modeInfo(QString("Inserting edge...Press again or press ESC to exit.")));
		op_mode_ = ADDEDGE;
	}
}


void RenderingWidget::SwitchToAddFace()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}

	if (op_mode_ == ADDFACE)
	{
		SwitchToNormal();
	}
	else
	{
		emit(AddEdgePressed(false));
		emit(AddFacePressed(true));
		emit(ChooseBoundPressed(false));
		emit(modeInfo(QString("Setting start edge...Press again or press ESC to exit.")));
		op_mode_ = ADDFACE;

		updateGL();
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
		switch (op_mode_)
		{
		case NORMAL:
			if (verts[i]->isFixed())
			{
				glColor3f(0.0, 1.0, 1.0);
			}
			if (bound_.size() >= N && bound_[i])
			{
				glColor3f(0.0, 0.0, 1.0);
			}

		case CHOOSEBOUND:
		case ADDEDGE:
			for (int j = 0; j < captured_verts_.size(); j++)
			{
				if (i == captured_verts_[j]->ID())
				{
					glColor3f(1.0, 0.0, 0.0);
					break;
				}
			}
			break;

		case ADDFACE:
			if (capturing_face_.is_captured_vert_[i])
			{
				glColor3f(1.0, 0.0, 0.0);
			}
			for (int j = 0; j < capturing_face_.corner_points_.size(); j++)
			{
				if (i == capturing_face_.corner_points_[j]->ID())
				{
					glColor3f(0.0, 1.0, 0.0);
				}
			}
			break;

		default:
			break;
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
			if (op_mode_ == ADDFACE)
			{
				if (capturing_face_.is_captured_edge_[i])
				{
					glColor3f(1.0, 0.0, 0.0);
				}
			}
			else
			{
				if (captured_edge_ == e)
				{
					glColor3f(1.0, 0.0, 0.0);
				}
				else
				if (ptr_frame_->isPillar(i))
				{
					glColor3f(0.0, 1.0, 1.0);
				}
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}

}


void RenderingWidget::DrawHeat(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == 0)
	{
		return;
	}

	const vector<int> Label = *(ptr_fiberprint_->GetLabel());
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

			int tag = (Label[i] % 6);
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


void RenderingWidget::DrawCut(bool bv)
{
	/*
	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == 0)
	{
		return;
	}

	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	const vector<int> cut = *(ptr_fiberprint_->GetCut());
	vector<bool> is_cut(M);
	fill(is_cut.begin(), is_cut.end(), false);
	for (int i = 0; i < cut.size(); i++)
	{
		is_cut[cut[i]] = true;
	}


	for (int i = 0; i < M; i++)
	{
		int j = edges[i]->ppair_->ID();
		if (i < j)
		{
			WF_edge *e = edges[i];
			WF_edge *e_pair = edges[i]->ppair_;

			if (is_cut[i])
			{
				glColor3f(1.0, 0.0, 0.0);
			}
			else
			{
			}
			glBegin(GL_LINE_LOOP);

			int tag = (Label[i] % 6);
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
	*/
}


void RenderingWidget::DrawBulk(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == NULL)
	{
		return;
	}

	const vector<DualVertex*> dual_vert = *(ptr_fiberprint_->GetDualVertList());
	const vector<BaseBulk*> bulk_list = *(ptr_fiberprint_->GetBulk());
	vector<vector<int>> range_state = *(ptr_fiberprint_->GetRangeState());
	const std::vector<WF_edge*>& edges = *(ptr_frame_->GetEdgeList());
	int M = ptr_frame_->SizeOfEdgeList();

	/*Draw Collision*/
	for (size_t i = 0; i < M; i++)
	{
		WF_edge *e = edges[i];
		WF_edge *e_pair = edges[i]->ppair_;

		if (e->ID() < e_pair->ID())
		{
			glBegin(GL_LINE_LOOP);

			//decide line color
			if (captured_edge_ != NULL)
			{
				int e_id = dual_vert[i]->dual_id();
				int cap_id = dual_vert[captured_edge_->ID()]->dual_id();
				if (captured_edge_->ID() == i)
				{
					glColor4f(0.0, 0.0, 1.0, 1);

					// draw bulk
					if (!has_lighting_)
					{
						glDepthMask(GL_FALSE);
					}
					
						BaseBulk *bulk = bulk_list[cap_id];
					if (bulk)
					{
						if (bulk->flag == 1)
						{
						bulk->Face(0)->Render(ptr_frame_, 0.1);
						bulk->Face(1)->Render(ptr_frame_, 0.6);
						bulk->Face(2)->Render(ptr_frame_, 0.2);
						bulk->Face(3)->Render(ptr_frame_, 0.2);
						bulk->Face(4)->Render(ptr_frame_, 0.35);
						bulk->Face(5)->Render(ptr_frame_, 0.35);
						bulk->Face(6)->Render(ptr_frame_, 0.4);
						bulk->Face(7)->Render(ptr_frame_, 0.4);
						bulk->Face(8)->Render(ptr_frame_, 0.3);
						bulk->Face(9)->Render(ptr_frame_, 0.3);
						bulk->Face(10)->Render(ptr_frame_, 0.3);
						bulk->Face(11)->Render(ptr_frame_, 0.5);
						bulk->Face(12)->Render(ptr_frame_, 0.3);
						bulk->Face(13)->Render(ptr_frame_, 0.3);

						}
						if (bulk->flag == 2)
						{
						
								bulk->Face(0)->Render(ptr_frame_, 0.1);
								bulk->Face(1)->Render(ptr_frame_, 0.1);
								bulk->Face(2)->Render(ptr_frame_, 0.1);
								bulk->Face(3)->Render(ptr_frame_, 0.1);
								bulk->Face(4)->Render(ptr_frame_, 0.1);
								bulk->Face(5)->Render(ptr_frame_, 0.1);
								bulk->Face(6)->Render(ptr_frame_, 0.5);
								bulk->Face(7)->Render(ptr_frame_, 0.5);
								bulk->Face(8)->Render(ptr_frame_, 0.3);
								bulk->Face(9)->Render(ptr_frame_, 0.3);
								bulk->Face(10)->Render(ptr_frame_, 0.3);
								bulk->Face(11)->Render(ptr_frame_, 0.5);
								bulk->Face(12)->Render(ptr_frame_, 0.4);		
								bulk->Face(13)->Render(ptr_frame_, 0.4);
								bulk->Face(14)->Render(ptr_frame_, 0.4);
						}
						
					}
					if (!has_lighting_)
					{
						glDepthMask(GL_TRUE);
					}
				}
				else
				if (range_state[cap_id][e_id] == 1)
				{
					glColor4f(0.00, 1, 0.00, 0.4);
				}
				else
				if (range_state[cap_id][e_id] == 2)
				{
					glColor4f(1.0, 0.0, 0.0, 1);
				}
				else
				{
					glColor4f(1.0, 1.0, 1.0, 1);
				}
			}
			else
			{
				glColor4f(1.0, 1.0, 1.0, 1);
			}

			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());

			glEnd();
		}
	}
}


void RenderingWidget::DrawOrder(bool bv)
{
	if (!bv || ptr_frame_ == NULL || ptr_fiberprint_ == NULL)
	{
		return;
	}

	const vector<DualVertex*> dual_vert = *(ptr_fiberprint_->GetDualVertList());

	if (op_mode_ == NORMAL)
	{
		const std::vector<QueueInfo> print_queue = *(ptr_fiberprint_->GetQueue());
		for (int i = 0; i < print_order_; i++)
		{
			int dual_id = print_queue[i].dual_id_;
			WF_edge *e = ptr_frame_->GetEdge(dual_vert[dual_id]->orig_id());
			glBegin(GL_LINE_LOOP);
			glColor3f(1.0, 1.0, 1.0);
			glVertex3fv(e->pvert_->RenderPos().data());
			glVertex3fv(e->ppair_->pvert_->RenderPos().data());
			glEnd();	
		}

		if (print_order_>0)
		{
			ptr_fiberprint_->ptr_seqanalyzer_->GetExtru(print_order_ - 1).Render(ptr_frame_, 0.5);
		}
		/*
		const std::vector<int> print_queue = *(ptr_fiberprint_->GetQueue());
		Orientation orientation = ptr_fiberprint_->ptr_seqanalyzer_->GetOrientation();
		int Nd = ptr_fiberprint_->ptr_graphcut_->ptr_dualgraph_->SizeOfVertList();

		if (orientation == SEQUENCE)
		{
			int max_order = min(print_order_, Nd);
			for (int i = 0; i < max_order; i++)
			{
				int ei = dual_vert[print_queue[i]]->orig_id();
				WF_edge *e = edges[ei];
				glBegin(GL_LINE_LOOP);
				glColor3f(1.0, 1.0, 1.0);
				glVertex3fv(e->pvert_->RenderPos().data());
				glVertex3fv(e->ppair_->pvert_->RenderPos().data());
				glEnd();
			}
		}
		else
		{
			if (print_order_ > 0)
			{
				int ei = dual_vert[print_queue[0]]->orig_id();
				WF_edge *e = edges[ei];
				glBegin(GL_LINE_LOOP);
				glColor3f(1.0, 1.0, 1.0);
				glVertex3fv(e->pvert_->RenderPos().data());
				glVertex3fv(e->ppair_->pvert_->RenderPos().data());
				glEnd();
			}

			int max_order = max(0, Nd - print_order_);
			for (int i = Nd - 1; i > max_order; i--)
			{
				int ei = dual_vert[print_queue[i]]->orig_id();
				WF_edge *e = edges[ei];
				glBegin(GL_LINE_LOOP);
				glColor3f(1.0, 1.0, 1.0);
				glVertex3fv(e->pvert_->RenderPos().data());
				glVertex3fv(e->ppair_->pvert_->RenderPos().data());
				glEnd();
			}
		}
		*/
		
	}
	/*
	else
	if (op_mode_ == SETSTART)
	{
		int M = ptr_frame_->SizeOfEdgeList();

		for (int i = 0; i < M; i++)
		{
			WF_edge *e = edges[i];
			WF_edge *e_pair = edges[i]->ppair_;

			if (e->ID() < e_pair->ID())
			{
				glBegin(GL_LINE_LOOP);

				if (captured_edge_ == i)
				{
					glColor3f(1.0, 0.0, 0.0);
				}
				else
				{
					glColor3f(1.0, 1.0, 1.0);
				}

				glVertex3fv(e->pvert_->RenderPos().data());
				glVertex3fv(e->ppair_->pvert_->RenderPos().data());

				glEnd();
			}
		}
	}
	*/
	//updateGL();

}


void RenderingWidget::FiberPrintAnalysis(double radius, double density, double g, double youngs_modulus, 
											double shear_modulus, double penalty, double D_tol, double pri_tol, 
											double dual_tol, double alpha, double beta, double gamma)
{
	/*
	ptr_frame_->RefineFrame();
	int N = ptr_frame_->SizeOfVertList();
	int M = ptr_frame_->SizeOfEdgeList();
	updateGL();
	*/
	
	//FiberPrintPARM *ptr_parm = new FiberPrintPARM(radius, density, g, youngs_modulus, shear_modulus, penalty,
	//												D_tol, pri_tol, dual_tol, alpha, beta, gamma);

	FiberPrintPARM *ptr_parm = new FiberPrintPARM();

	delete ptr_fiberprint_;
	ptr_fiberprint_ = new FiberPrintPlugIn(ptr_frame_, ptr_parm);
	ptr_fiberprint_->Print();

	emit(SetOrderSlider(0));
	emit(SetMaxOrderSlider(ptr_fiberprint_->ptr_graphcut_->ptr_dualgraph_->SizeOfVertList()));

	delete ptr_parm;
}


void RenderingWidget::PrintLayer(int layer)
{
	print_layer_ = layer;
	updateGL();
}


void RenderingWidget::PrintOrder(int order)
{
	print_order_ = order;
	updateGL();
}


void RenderingWidget::SimplifyFrame()
{
	if (ptr_frame_ == NULL)
	{
		return;
	}
	ptr_frame_->SimplifyFrame();
	is_simplified_ = true;

	emit(modeInfo(QString("Insert edge (I) | Choose boundary (C)")));
	emit(operatorInfo(QString("")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

	updateGL();
}


void RenderingWidget::ProjectBound()
{
	if (bound_.size() <= 0)
	{
		return;
	}
	ptr_frame_->ProjectBound(&bound_);
	op_mode_ = NORMAL;
	emit(modeInfo(QString("Insert edge (I) | Choose boundary (C)")));
	emit(operatorInfo(QString("")));
	emit(meshInfo(ptr_frame_->SizeOfVertList(), ptr_frame_->SizeOfEdgeList()));

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

