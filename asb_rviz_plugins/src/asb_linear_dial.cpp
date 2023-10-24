//
// Created by enrico on 21/10/23.
//

#include "include/asb_rviz_plugins/asb_linear_dial.h"

//#include "qwt_thermo.h"
#include "qwt_scale_engine.h"
#include "qwt_scale_draw.h"
#include "qwt_scale_map.h"
#include "qwt_color_map.h"
#include <qpainter.h>
#include <qevent.h>
#include <qdrawutil.h>
#include <qstyle.h>
#include <qstyleoption.h>
#include <qmath.h>
#include <QPainterPath>

static inline void qwtDrawLine(QPainter *painter, int pos,
                               const QColor &color, const QRect &pipeRect, const QRect &liquidRect,
                               Qt::Orientation orientation) {
  painter->setPen(color);
  if (orientation == Qt::Horizontal) {
    if (pos >= liquidRect.left() && pos < liquidRect.right())
      painter->drawLine(pos, pipeRect.top(), pos, pipeRect.bottom());
  } else {
    if (pos >= liquidRect.top() && pos < liquidRect.bottom())
      painter->drawLine(pipeRect.left(), pos, pipeRect.right(), pos);
  }
}

QVector<double> qwtTickList(const QwtScaleDiv &scaleDiv) {
  QVector<double> values;

  double lowerLimit = scaleDiv.interval().minValue();
  double upperLimit = scaleDiv.interval().maxValue();

  if (upperLimit < lowerLimit)
    qSwap(lowerLimit, upperLimit);

  values += lowerLimit;

  for (int tickType = QwtScaleDiv::MinorTick;
       tickType < QwtScaleDiv::NTickTypes; tickType++) {
    const QList<double> ticks = scaleDiv.ticks(tickType);

    for (int i = 0; i < ticks.count(); i++) {
      const double v = ticks[i];
      if (v > lowerLimit && v < upperLimit)
        values += v;
    }
  }

  values += upperLimit;

  return values;
}

class ASBThermo::PrivateData {
public:
  PrivateData() :
          orientation(Qt::Vertical),
          scalePosition(ASBThermo::TrailingScale),
          spacing(3),
          borderWidth(2),
          pipeWidth(10),
          upperAlarmLevel(0.0),
          upperAlarmEnabled(false),
          lowerAlarmLevel(0.0),
          lowerAlarmEnabled(false),
          autoFillPipe(true),
          originMode(ASBThermo::OriginMinimum),
          origin(0.0),
          colorMap(nullptr),
          value(0.0),
          setpointValue(0.0),
          setpointEnabled(false) {
    rangeFlags = QwtInterval::IncludeBorders;
  }

  ~PrivateData() {
    delete colorMap;
  }

  Qt::Orientation orientation;
  ASBThermo::ScalePosition scalePosition;

  int spacing;
  int borderWidth;
  int pipeWidth;

  QwtInterval::BorderFlags rangeFlags;
  double upperAlarmLevel;
  bool upperAlarmEnabled;
  double lowerAlarmLevel;
  bool lowerAlarmEnabled;
  bool autoFillPipe;
  ASBThermo::OriginMode originMode;
  double origin;

  QwtColorMap *colorMap;

  double value;
  double setpointValue;
  bool setpointEnabled;
};

/*!
  Constructor
  \param parent Parent widget
*/
ASBThermo::ASBThermo(QWidget *parent) :
        QwtAbstractScale(parent) {
  d_data = new PrivateData;

  QSizePolicy policy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  if (d_data->orientation == Qt::Vertical)
    policy.transpose();

  setSizePolicy(policy);

  setAttribute(Qt::WA_WState_OwnSizePolicy, false);
  layoutThermo(true);
}

//! Destructor
ASBThermo::~ASBThermo() {
  delete d_data;
}

/*!
  \brief Exclude/Include min/max values

  According to the flags minValue() and maxValue()
  are included/excluded from the pipe. In case of an
  excluded value the corresponding tick is painted
  1 pixel off of the pipeRect().

  F.e. when a minimum
  of 0.0 has to be displayed as an empty pipe the minValue()
  needs to be excluded.

  \param flags Range flags
  \sa rangeFlags()
*/
void ASBThermo::setRangeFlags(QwtInterval::BorderFlags flags) {
  if (d_data->rangeFlags != flags) {
    d_data->rangeFlags = flags;
    update();
  }
}

/*!
  \return Range flags
  \sa setRangeFlags()
*/
QwtInterval::BorderFlags ASBThermo::rangeFlags() const {
  return d_data->rangeFlags;
}

/*!
  Set the current value.

  \param value New Value
  \sa value()
*/
void ASBThermo::setValue(double value) {
  if (d_data->value != value) {
    d_data->value = value;
    update();
  }
}

//! Return the value.
double ASBThermo::value() const {
  return d_data->value;
}

/*!
  Set the current setpoint value.

  \param setpointValue New setpointValue
  \sa setpointValue()
*/
void ASBThermo::setSetpointValue(double setpointValue) {
  if (d_data->setpointValue != setpointValue) {
    d_data->setpointValue = setpointValue;
    update();
  }
}

//! Return the setpoint value.
double ASBThermo::setpointValue() const {
  return d_data->setpointValue;
}

/*!
  Enables or disables the setpoint.

  \param setpointValue New setpointValue
  \sa setpointValue()
*/
void ASBThermo::setSetpointEnabled(bool setpointEnabled) {
  if (d_data->setpointEnabled != setpointEnabled) {
    d_data->setpointEnabled = setpointEnabled;
    update();
  }
}

//! Return the setpoint enabled.
bool ASBThermo::setpointEnabled() const {
  return d_data->setpointEnabled;
}

/*!
  \brief Set a scale draw

  For changing the labels of the scales, it
  is necessary to derive from QwtScaleDraw and
  overload QwtScaleDraw::label().

  \param scaleDraw ScaleDraw object, that has to be created with
                   new and will be deleted in ~ASBThermo() or the next
                   call of setScaleDraw().
*/
void ASBThermo::setScaleDraw(QwtScaleDraw *scaleDraw) {
  setAbstractScaleDraw(scaleDraw);
  layoutThermo(true);
}

/*!
   \return the scale draw of the thermo
   \sa setScaleDraw()
*/
const QwtScaleDraw *ASBThermo::scaleDraw() const {
  return static_cast<const QwtScaleDraw *>( abstractScaleDraw());
}

/*!
   \return the scale draw of the thermo
   \sa setScaleDraw()
*/
QwtScaleDraw *ASBThermo::scaleDraw() {
  return static_cast<QwtScaleDraw *>( abstractScaleDraw());
}

/*!
  Paint event handler
  \param event Paint event
*/
void ASBThermo::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setClipRegion(event->region());

  QStyleOption opt;
  opt.init(this);
  style()->drawPrimitive(QStyle::PE_Widget, &opt, &painter, this);

  const QRect tRect = pipeRect();

  if (!tRect.contains(event->rect())) {
    if (d_data->scalePosition != ASBThermo::NoScale)
      scaleDraw()->draw(&painter, palette());
  }

  const int bw = d_data->borderWidth;

  const QBrush brush = palette().brush(QPalette::Base);
  qDrawShadePanel(&painter,
                  tRect.adjusted(-bw, -bw, bw, bw),
                  palette(), true, bw,
                  d_data->autoFillPipe ? &brush : nullptr);

  QRect alarmPipeRect = tRect.adjusted(-d_data->spacing - bw, 0, -d_data->spacing - bw, 0);
  alarmPipeRect.setWidth(d_data->spacing);
  drawAlarmPipe(&painter, alarmPipeRect);

  drawLiquid(&painter, tRect);
}

/*!
  Resize event handler
  \param event Resize event
*/
void ASBThermo::resizeEvent(QResizeEvent *event) {
  Q_UNUSED(event);
  layoutThermo(false);
}

/*!
  Qt change event handler
  \param event Event
*/
void ASBThermo::changeEvent(QEvent *event) {
  switch (event->type()) {
    case QEvent::StyleChange:
    case QEvent::FontChange: {
      layoutThermo(true);
      break;
    }
    default:
      break;
  }
}

/*!
  Recalculate the ASBThermo geometry and layout based on
  pipeRect() and the fonts.

  \param update_geometry notify the layout system and call update
         to redraw the scale
*/
void ASBThermo::layoutThermo(bool update_geometry) {
  const QRect tRect = pipeRect();
  const int bw = d_data->borderWidth + d_data->spacing;
  const bool inverted = (upperBound() < lowerBound());

  int from, to;

  if (d_data->orientation == Qt::Horizontal) {
    from = tRect.left();
    to = tRect.right();

    if (d_data->rangeFlags & QwtInterval::ExcludeMinimum) {
      if (inverted)
        to++;
      else
        from--;
    }
    if (d_data->rangeFlags & QwtInterval::ExcludeMaximum) {
      if (inverted)
        from--;
      else
        to++;
    }

    if (d_data->scalePosition == ASBThermo::TrailingScale) {
      scaleDraw()->setAlignment(QwtScaleDraw::TopScale);
      scaleDraw()->move(from, tRect.top() - bw);
    } else {
      scaleDraw()->setAlignment(QwtScaleDraw::BottomScale);
      scaleDraw()->move(from, tRect.bottom() + bw);
    }

    scaleDraw()->setLength(qMax(to - from, 0));
  } else // Qt::Vertical
  {
    from = tRect.top();
    to = tRect.bottom();

    if (d_data->rangeFlags & QwtInterval::ExcludeMinimum) {
      if (inverted)
        from--;
      else
        to++;
    }
    if (d_data->rangeFlags & QwtInterval::ExcludeMaximum) {
      if (inverted)
        to++;
      else
        from--;
    }

    if (d_data->scalePosition == ASBThermo::LeadingScale) {
      scaleDraw()->setAlignment(QwtScaleDraw::RightScale);
      scaleDraw()->move(tRect.right() + bw, from);
    } else {
      scaleDraw()->setAlignment(QwtScaleDraw::LeftScale);
      scaleDraw()->move(tRect.left() - bw, from);
    }

    scaleDraw()->setLength(qMax(to - from, 0));
  }

  if (update_geometry) {
    updateGeometry();
    update();
  }
}

/*!
  \return Bounding rectangle of the pipe ( without borders )
          in widget coordinates
*/
QRect ASBThermo::pipeRect() const {
  int mbd = 0;
  if (d_data->scalePosition != ASBThermo::NoScale) {
    int d1, d2;
    scaleDraw()->getBorderDistHint(font(), d1, d2);
    mbd = qMax(d1, d2);
  }
  const int bw = d_data->borderWidth;
  const int scaleOff = bw + mbd;

  const QRect cr = contentsRect();

  QRect pipeRect = cr;
  if (d_data->orientation == Qt::Horizontal) {
    pipeRect.adjust(scaleOff, 0, -scaleOff, 0);

    if (d_data->scalePosition == ASBThermo::TrailingScale)
      pipeRect.setTop(cr.top() + cr.height() - bw - d_data->pipeWidth);
    else
      pipeRect.setTop(bw);

    pipeRect.setHeight(d_data->pipeWidth);
  } else // Qt::Vertical
  {
    pipeRect.adjust(0, scaleOff, 0, -scaleOff);

    if (d_data->scalePosition == ASBThermo::LeadingScale)
      pipeRect.setLeft(bw);
    else
      pipeRect.setLeft(cr.left() + cr.width() - bw - d_data->pipeWidth);

    pipeRect.setWidth(d_data->pipeWidth);
  }

  return pipeRect;
}

/*!
  \brief Set the orientation.
  \param orientation Allowed values are Qt::Horizontal and Qt::Vertical.

  \sa orientation(), scalePosition()
*/
void ASBThermo::setOrientation(Qt::Orientation orientation) {
  if (orientation == d_data->orientation)
    return;

  d_data->orientation = orientation;

  if (!testAttribute(Qt::WA_WState_OwnSizePolicy)) {
    QSizePolicy sp = sizePolicy();
    sp.transpose();
    setSizePolicy(sp);

    setAttribute(Qt::WA_WState_OwnSizePolicy, false);
  }

  layoutThermo(true);
}

/*!
  \return Orientation
  \sa setOrientation()
*/
Qt::Orientation ASBThermo::orientation() const {
  return d_data->orientation;
}

/*!
  \brief Change how the origin is determined.
  \sa originMode(), serOrigin(), origin()
 */
void ASBThermo::setOriginMode(OriginMode m) {
  if (m == d_data->originMode)
    return;

  d_data->originMode = m;
  update();
}

/*!
  \return Mode, how the origin is determined.
  \sa setOriginMode(), serOrigin(), origin()
 */
ASBThermo::OriginMode ASBThermo::originMode() const {
  return d_data->originMode;
}

/*!
  \brief Specifies the custom origin.

  If originMode is set to OriginCustom this property controls where the
  liquid starts.

  \param origin New origin level
  \sa setOriginMode(), originMode(), origin()
 */
void ASBThermo::setOrigin(double origin) {
  if (origin == d_data->origin)
    return;

  d_data->origin = origin;
  update();
}

/*!
  \return Origin of the thermo, when OriginCustom is enabled
  \sa setOrigin(), setOriginMode(), originMode()
 */
double ASBThermo::origin() const {
  return d_data->origin;
}

/*!
  \brief Change the position of the scale
  \param scalePosition Position of the scale.

  \sa ScalePosition, scalePosition()
*/
void ASBThermo::setScalePosition(ScalePosition scalePosition) {
  if (d_data->scalePosition == scalePosition)
    return;

  d_data->scalePosition = scalePosition;

  if (testAttribute(Qt::WA_WState_Polished))
    layoutThermo(true);
}

/*!
   \return Scale position.
   \sa setScalePosition()
*/
ASBThermo::ScalePosition ASBThermo::scalePosition() const {
  return d_data->scalePosition;
}

//! Notify a scale change.
void ASBThermo::scaleChange() {
  layoutThermo(true);
}

/*!
   Redraw the alarm region beside the thermometer pipe.
   \param painter Painter
   \param drawRegionRect Bounding rectangle of the alarm region
*/
void ASBThermo::drawAlarmPipe(
        QPainter *painter, const QRect &alarmPipeRect) const {
  painter->save();
  painter->setClipRect(alarmPipeRect, Qt::IntersectClip);
  painter->setPen(Qt::NoPen);

  QRect filledUpperAlarmRect = upperAlarmPipeRegionRect(alarmPipeRect);
  if (!filledUpperAlarmRect.isEmpty() && d_data->upperAlarmEnabled) {
    painter->fillRect(filledUpperAlarmRect, palette().brush(QPalette::HighlightedText));
  }

  QRect filledLowerAlarmRect = lowerAlarmPipeRegionRect(alarmPipeRect);
  if (!filledLowerAlarmRect.isEmpty() && d_data->lowerAlarmEnabled) {
    painter->fillRect(filledLowerAlarmRect, palette().brush(QPalette::HighlightedText));
  }

  painter->restore();
}

/*!
   Redraw the liquid in thermometer pipe.
   \param painter Painter
   \param pipeRect Bounding rectangle of the pipe without borders
*/
void ASBThermo::drawLiquid(
        QPainter *painter, const QRect &pipeRect) const {
  painter->save();
  painter->setClipRect(pipeRect, Qt::IntersectClip);
  painter->setPen(Qt::NoPen);

  const QwtScaleMap scaleMap = scaleDraw()->scaleMap();

  QRect liquidRect = fillRect(pipeRect);

  if (d_data->colorMap != nullptr) {
    const QwtInterval interval = scaleDiv().interval().normalized();

    // Because the positions of the ticks are rounded
    // we calculate the colors for the rounded tick values

    QVector<double> values = qwtTickList(scaleDraw()->scaleDiv());

    if (scaleMap.isInverting())
      std::sort(values.begin(), values.end(), std::greater<double>());
    else
      std::sort(values.begin(), values.end(), std::less<double>());

    int from;
    if (!values.isEmpty()) {
      from = qRound(scaleMap.transform(values[0]));
      qwtDrawLine(painter, from,
                  d_data->colorMap->color(interval, values[0]),
                  pipeRect, liquidRect, d_data->orientation);
    }

    for (int i = 1; i < values.size(); i++) {
      const int to = qRound(scaleMap.transform(values[i]));

      for (int pos = from + 1; pos < to; pos++) {
        const double v = scaleMap.invTransform(pos);

        qwtDrawLine(painter, pos,
                    d_data->colorMap->color(interval, v),
                    pipeRect, liquidRect, d_data->orientation);
      }

      qwtDrawLine(painter, to,
                  d_data->colorMap->color(interval, values[i]),
                  pipeRect, liquidRect, d_data->orientation);

      from = to;
    }
  } else {

    if (d_data->upperAlarmEnabled && (d_data->value > d_data->upperAlarmLevel)) {
      painter->fillRect(liquidRect, palette().brush(QPalette::Highlight));
    } else if (d_data->lowerAlarmEnabled && (d_data->value < d_data->lowerAlarmLevel)) {
      painter->fillRect(liquidRect, palette().brush(QPalette::Highlight));
    } else {
      painter->fillRect(liquidRect, palette().brush(QPalette::ButtonText));
    }

//  Draw setpoint triangle
    if (d_data->setpointEnabled) {

      int setpointScaleValue = qRound(scaleMap.transform(d_data->setpointValue));

      painter->setRenderHint( QPainter::Antialiasing );
      QRectF setpointRect = QRectF(liquidRect.left() + liquidRect.width() / 4. + 1,
                                   setpointScaleValue - liquidRect.width()/2.,
                                   3*liquidRect.width()/4. - 1,
                                   liquidRect.width());
      QPainterPath setpointPath;
      setpointPath.moveTo(setpointRect.left(), setpointRect.top() + setpointRect.height() / 2);
      setpointPath.lineTo(setpointRect.topRight());
      setpointPath.lineTo(setpointRect.bottomRight());
      setpointPath.lineTo(setpointRect.left(), setpointRect.top() + setpointRect.height() / 2);
      painter->fillPath(setpointPath, palette().brush(QPalette::Base));
      painter->strokePath(setpointPath, QPen(palette().brush(QPalette::Dark), 1.5));
    }

  }

  painter->restore();
}

/*!
  \brief Change the spacing between pipe and scale

  A spacing of 0 means, that the backbone of the scale is below
  the pipe.

  The default setting is 3 pixels.

  \param spacing Number of pixels
  \sa spacing();
*/
void ASBThermo::setSpacing(int spacing) {
  if (spacing <= 0)
    spacing = 0;

  if (spacing != d_data->spacing) {
    d_data->spacing = spacing;
    layoutThermo(true);
  }
}

/*!
  \return Number of pixels between pipe and scale
  \sa setSpacing()
*/
int ASBThermo::spacing() const {
  return d_data->spacing;
}

/*!
   Set the border width of the pipe.
   \param width Border width
   \sa borderWidth()
*/
void ASBThermo::setBorderWidth(int width) {
  if (width <= 0)
    width = 0;

  if (width != d_data->borderWidth) {
    d_data->borderWidth = width;
    layoutThermo(true);
  }
}

/*!
   \return Border width of the thermometer pipe.
   \sa setBorderWidth()
*/
int ASBThermo::borderWidth() const {
  return d_data->borderWidth;
}

/*!
  \brief Assign a color map for the fill color

  \param colorMap Color map
  \warning The alarm threshold has no effect, when
           a color map has been assigned
*/
void ASBThermo::setColorMap(QwtColorMap *colorMap) {
  if (colorMap != d_data->colorMap) {
    delete d_data->colorMap;
    d_data->colorMap = colorMap;
  }
}

/*!
  \return Color map for the fill color
  \warning The alarm threshold has no effect, when
           a color map has been assigned
*/
QwtColorMap *ASBThermo::colorMap() {
  return d_data->colorMap;
}

/*!
  \return Color map for the fill color
  \warning The alarm threshold has no effect, when
           a color map has been assigned
*/
const QwtColorMap *ASBThermo::colorMap() const {
  return d_data->colorMap;
}

/*!
  \brief Change the brush of the liquid.

  Changes the QPalette::ButtonText brush of the palette.

  \param brush New brush.
  \sa fillBrush(), QWidget::setPalette()
*/
void ASBThermo::setFillBrush(const QBrush &brush) {
  QPalette pal = palette();
  pal.setBrush(QPalette::ButtonText, brush);
  setPalette(pal);
}

/*!
  \return Liquid ( QPalette::ButtonText ) brush.
  \sa setFillBrush(), QWidget::palette()
*/
QBrush ASBThermo::fillBrush() const {
  return palette().brush(QPalette::ButtonText);
}

/*!
  \brief Specify the liquid brush above or below the alarm thresholds

  Changes the QPalette::Highlight brush of the palette.

  \param brush New brush.
  \sa alarmBrush(), QWidget::setPalette()

  \warning The alarm thresholds have no effect, when
           a color map has been assigned
*/
void ASBThermo::setAlarmBrush(const QBrush &brush) {
  QPalette pal = palette();
  pal.setBrush(QPalette::Highlight, brush);
  setPalette(pal);
}

/*!
  \return Liquid brush ( QPalette::Highlight ) above or below the alarm thresholds.
  \sa setAlarmBrush(), QWidget::palette()

  \warning The alarm thresholds have no effect, when
           a color map has been assigned
*/
QBrush ASBThermo::alarmBrush() const {
  return palette().brush(QPalette::Highlight);
}

/*!
  Specify the upper alarm threshold.

  \param level upper alarm threshold
  \sa upperAlarmLevel()

  \warning The alarm thresholds have no effect, when
           a color map has been assigned
*/
void ASBThermo::setUpperAlarmLevel(double level) {
  d_data->upperAlarmLevel = level;
  d_data->upperAlarmEnabled = true;
  update();
}

/*!
  \return Alarm threshold.
  \sa setUpperAlarmLevel()

  \warning The alarm threshold has no effect, when
           a color map has been assigned
*/
double ASBThermo::upperAlarmLevel() const {
  return d_data->upperAlarmLevel;
}

/*!
  Specify the lower alarm threshold.

  \param level lower alarm threshold
  \sa lowerAlarmLevel()

  \warning The alarm thresholds have no effect, when
           a color map has been assigned
*/
void ASBThermo::setLowerAlarmLevel(double level) {
  d_data->lowerAlarmLevel = level;
  d_data->lowerAlarmEnabled = true;
  update();
}

/*!
  \return Alarm threshold.
  \sa setLowerAlarmLevel()

  \warning The alarm threshold has no effect, when
           a color map has been assigned
*/
double ASBThermo::lowerAlarmLevel() const {
  return d_data->lowerAlarmLevel;
}

/*!
  Change the width of the pipe.

  \param width Width of the pipe
  \sa pipeWidth()
*/
void ASBThermo::setPipeWidth(int width) {
  if (width > 0) {
    d_data->pipeWidth = width;
    layoutThermo(true);
  }
}

/*!
  \return Width of the pipe.
  \sa setPipeWidth()
*/
int ASBThermo::pipeWidth() const {
  return d_data->pipeWidth;
}

/*!
  \brief Enable or disable the upper alarm threshold
  \param on true (disabled) or false (enabled)

  \warning The upper alarm threshold has no effect, when
           a color map has been assigned
*/
void ASBThermo::setUpperAlarmEnabled(bool on) {
  d_data->upperAlarmEnabled = on;
  update();
}

/*!
  \return True, when the upper alarm threshold is enabled.

  \warning The upper alarm threshold has no effect, when
           a color map has been assigned
*/
bool ASBThermo::upperAlarmEnabled() const {
  return d_data->upperAlarmEnabled;
}

/*!
  \brief Enable or disable the lower alarm threshold
  \param on true (disabled) or false (enabled)

  \warning The lower alarm threshold has no effect, when
           a color map has been assigned
*/
void ASBThermo::setLowerAlarmEnabled(bool on) {
  d_data->lowerAlarmEnabled = on;
  update();
}

/*!
  \return True, when the lower alarm threshold is enabled.

  \warning The lower alarm threshold has no effect, when
           a color map has been assigned
*/
bool ASBThermo::lowerAlarmEnabled() const {
  return d_data->lowerAlarmEnabled;
}

/*!
  \return the minimum size hint
  \sa minimumSizeHint()
*/
QSize ASBThermo::sizeHint() const {
  return minimumSizeHint();
}

/*!
  \return Minimum size hint
  \warning The return value depends on the font and the scale.
  \sa sizeHint()
*/
QSize ASBThermo::minimumSizeHint() const {
  int w = 0, h = 0;

  if (d_data->scalePosition != NoScale) {
    const int sdExtent = qCeil(scaleDraw()->extent(font()));
    const int sdLength = scaleDraw()->minLength(font());

    w = sdLength;
    h = d_data->pipeWidth + sdExtent + d_data->spacing;

  } else // no scale
  {
    w = 200;
    h = d_data->pipeWidth;
  }

  if (d_data->orientation == Qt::Vertical)
    qSwap(w, h);

  w += 2 * d_data->borderWidth;
  h += 2 * d_data->borderWidth;

  // finally add the margins
//    int left, right, top, bottom;
//    getContentsMargins(&left, &top, &right, &bottom);
  auto margins = contentsMargins();
  w += margins.left() + margins.right();
  h += margins.top() + margins.bottom();

  return QSize(w, h);
}

/*!
  \brief Calculate the filled rectangle of the pipe

  \param pipeRect Rectangle of the pipe
  \return Rectangle to be filled ( fill and alarm brush )

  \sa pipeRect(), alarmRect()
 */
QRect ASBThermo::fillRect(const QRect &pipeRect) const {
  double origin;
  if (d_data->originMode == OriginMinimum) {
    origin = qMin(lowerBound(), upperBound());
  } else if (d_data->originMode == OriginMaximum) {
    origin = qMax(lowerBound(), upperBound());
  } else // OriginCustom
  {
    origin = d_data->origin;
  }

  const QwtScaleMap scaleMap = scaleDraw()->scaleMap();

  int from = qRound(scaleMap.transform(d_data->value));
  int to = qRound(scaleMap.transform(origin));

  if (to < from)
    qSwap(from, to);

  QRect fillRect = pipeRect;
  if (d_data->orientation == Qt::Horizontal) {
    fillRect.setLeft(from);
    fillRect.setRight(to);
  } else // Qt::Vertical
  {
    fillRect.setTop(from);
    fillRect.setBottom(to);
  }

  return fillRect.normalized();
}

/*!
  \brief Calculate the upper alarm region rectangle of the alarm pipe

  \param fillRect Filled rectangle in the alarm pipe
  \return Rectangle to be filled with the alarm brush

  \sa pipeRect(), fillRect(), upperAlarmLevel(), alarmBrush()
 */
QRect ASBThermo::upperAlarmPipeRegionRect(const QRect &fillRect) const {
  QRect alarmRect(0, 0, -1, -1); // something invalid

  if (!d_data->upperAlarmEnabled)
    return alarmRect;

  const bool inverted = (upperBound() < lowerBound());

  const QwtScaleMap map = scaleDraw()->scaleMap();
  const int upperAlarmPos = qRound(map.transform(d_data->upperAlarmLevel));

  if (d_data->orientation == Qt::Horizontal) {
    int v1, v2;
    if (inverted) {
      v1 = fillRect.left();
      v2 = upperAlarmPos - 1;
    } else {
      v1 = upperAlarmPos + 1;
      v2 = fillRect.right();
    }
    alarmRect.setRect(v1, fillRect.top(), v2 - v1 + 1, fillRect.height());
  } else {
    int v1, v2;
    if (inverted) {
      v1 = upperAlarmPos + 1;
      v2 = fillRect.bottom();
    } else {
      v1 = fillRect.top();
      v2 = upperAlarmPos - 1;
    }
    alarmRect.setRect(fillRect.left(), v1, fillRect.width(), v2 - v1 + 1);
  }

  return alarmRect;
}

/*!
  \brief Calculate the lower alarm region rectangle of the alarm pipe

  \param fillRect Filled rectangle in the alarm pipe
  \return Rectangle to be filled with the alarm brush

  \sa pipeRect(), fillRect(), lowerAlarmLevel(), alarmBrush()
 */
QRect ASBThermo::lowerAlarmPipeRegionRect(const QRect &fillRect) const {
  QRect alarmRect(0, 0, -1, -1); // something invalid

  if (!d_data->lowerAlarmEnabled)
    return alarmRect;

  const bool inverted = (upperBound() < lowerBound());

  const QwtScaleMap map = scaleDraw()->scaleMap();
  const int lowerAlarmPos = qRound(map.transform(d_data->lowerAlarmLevel));

  if (d_data->orientation == Qt::Horizontal) {
    int v1, v2;
    if (inverted) {
      v1 = lowerAlarmPos + 1;
      v2 = fillRect.right();
    } else {
      v1 = fillRect.left();
      v2 = lowerAlarmPos - 1;
    }
    alarmRect.setRect(v1, fillRect.top(), v2 - v1 + 1, fillRect.height());
  } else {
    int v1, v2;
    if (inverted) {
      v1 = fillRect.top();
      v2 = lowerAlarmPos - 1;
    } else {
      v1 = lowerAlarmPos + 1;
      v2 = fillRect.bottom();
    }
    alarmRect.setRect(fillRect.left(), v1, fillRect.width(), v2 - v1 + 1);
  }

  return alarmRect;
}
