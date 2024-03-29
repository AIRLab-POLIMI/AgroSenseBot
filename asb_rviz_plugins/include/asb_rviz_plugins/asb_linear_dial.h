//
// Created by enrico on 21/10/23.
//

#ifndef ASB_RVIZ_PLUGINS_ASB_LINEAR_DIAL_H
#define ASB_RVIZ_PLUGINS_ASB_LINEAR_DIAL_H

#include "qwt_global.h"
#include "qwt_abstract_scale.h"
#include "qwt_interval.h"

class QwtScaleDraw;
class QwtColorMap;

/*!
  \brief The Thermometer Widget

  ASBThermo is a widget which displays a value in an interval. It supports:
  - a horizontal or vertical layout;
  - a range;
  - a scale;
  - upper and lower alarm levels
  - a set point.

  \image html sysinfo.png

  The fill colors might be calculated from an optional color map
  If no color map has been assigned ASBThermo uses the
  following colors/brushes from the widget palette:

  - QPalette::Base
    Background of the pipe
  - QPalette::ButtonText
    Fill brush below the alarm level
  - QPalette::Highlight
    Fill brush above upper or below lower alarm levels
  - QPalette::HighlightedText
    For the upper and lower alarm level lines
  - QPalette::Dark
    For the set point indicator contour line
  - QPalette::WindowText
    For the axis of the scale
  - QPalette::Text
    For the labels of the scale
*/
class QWT_EXPORT ASBThermo: public QwtAbstractScale
{
Q_OBJECT

  Q_ENUMS( ScalePosition )
  Q_ENUMS( OriginMode )

  Q_PROPERTY( Qt::Orientation orientation
                      READ orientation WRITE setOrientation )
  Q_PROPERTY( ScalePosition scalePosition
                      READ scalePosition WRITE setScalePosition )
  Q_PROPERTY( OriginMode originMode READ originMode WRITE setOriginMode )

  Q_PROPERTY( bool upperAlarmEnabled READ upperAlarmEnabled WRITE setUpperAlarmEnabled )
  Q_PROPERTY( double upperAlarmLevel READ upperAlarmLevel WRITE setUpperAlarmLevel )
  Q_PROPERTY( bool lowerAlarmEnabled READ lowerAlarmEnabled WRITE setLowerAlarmEnabled )
  Q_PROPERTY( double lowerAlarmLevel READ lowerAlarmLevel WRITE setLowerAlarmLevel )
  Q_PROPERTY( double origin READ origin WRITE setOrigin )
  Q_PROPERTY( int spacing READ spacing WRITE setSpacing )
  Q_PROPERTY( int borderWidth READ borderWidth WRITE setBorderWidth )
  Q_PROPERTY( int pipeWidth READ pipeWidth WRITE setPipeWidth )
  Q_PROPERTY( double value READ value WRITE setValue )
  Q_PROPERTY( double setpointValue READ setpointValue WRITE setSetpointValue )
  Q_PROPERTY( double setpointEnabled READ setpointEnabled WRITE setSetpointEnabled )

public:

  /*!
    Position of the scale
    \sa setScalePosition(), setOrientation()
   */
  enum ScalePosition
  {
    //! The slider has no scale
    NoScale,

    //! The scale is right of a vertical or below of a horizontal slider
    LeadingScale,

    //! The scale is left of a vertical or above of a horizontal slider
    TrailingScale
  };

  /*!
    Origin mode. This property specifies where the beginning of the liquid
    is placed.

    \sa setOriginMode(), setOrigin()
  */
  enum OriginMode
  {
    //! The origin is the minimum of the scale
    OriginMinimum,

    //! The origin is the maximum of the scale
    OriginMaximum,

    //! The origin is specified using the origin() property
    OriginCustom
  };

  explicit ASBThermo( QWidget *parent = NULL );
  virtual ~ASBThermo();

  void setOrientation( Qt::Orientation );
  Qt::Orientation orientation() const;

  void setScalePosition( ScalePosition );
  ScalePosition scalePosition() const;

  void setSpacing( int );
  int spacing() const;

  void setBorderWidth( int );
  int borderWidth() const;

  void setOriginMode( OriginMode );
  OriginMode originMode() const;

  void setOrigin( double );
  double origin() const;

  void setFillBrush( const QBrush & );
  QBrush fillBrush() const;

  void setAlarmBrush( const QBrush & );
  QBrush alarmBrush() const;

  void setUpperAlarmLevel(double );
  double upperAlarmLevel() const;

  void setUpperAlarmEnabled(bool );
  bool upperAlarmEnabled() const;

  void setLowerAlarmLevel(double );
  double lowerAlarmLevel() const;

  void setLowerAlarmEnabled(bool );
  bool lowerAlarmEnabled() const;

  void setColorMap( QwtColorMap * );
  QwtColorMap *colorMap();
  const QwtColorMap *colorMap() const;

  void setPipeWidth( int );
  int pipeWidth() const;

  void setRangeFlags( QwtInterval::BorderFlags );
  QwtInterval::BorderFlags rangeFlags() const;

  double value() const;

  double setpointValue() const;

  bool setpointEnabled() const;

  virtual QSize sizeHint() const;
  virtual QSize minimumSizeHint() const;

  void setScaleDraw( QwtScaleDraw * );
  const QwtScaleDraw *scaleDraw() const;

public Q_SLOTS:
  virtual void setValue( double );

  virtual void setSetpointValue( double );

  virtual void setSetpointEnabled( bool );

protected:
  virtual void drawLiquid( QPainter *, const QRect & ) const;

  virtual void drawAlarmPipe(QPainter *painter, const QRect &alarmPipeRect) const;

  virtual void scaleChange();

  virtual void paintEvent( QPaintEvent * );
  virtual void resizeEvent( QResizeEvent * );
  virtual void changeEvent( QEvent * );

  QwtScaleDraw *scaleDraw();

  QRect pipeRect() const;
  QRect fillRect( const QRect & ) const;
  QRect upperAlarmPipeRegionRect(const QRect &fillRect) const;
  QRect lowerAlarmPipeRegionRect(const QRect &fillRect) const;

private:
  void layoutThermo( bool );

  class PrivateData;
  PrivateData *d_data;
};

#endif //ASB_RVIZ_PLUGINS_ASB_LINEAR_DIAL_H
