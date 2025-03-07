/*
  Optimal Visual Servoing
  Copyright (C) 2018  Siddharth Jha, Aashay Bhise

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#pragma once
#include <cstdlib>
#include <vector>
#include <iostream>
#include <ceres/ceres.h>
#include <ugv_visual_servoing/DynamicWindowSampler.h>

struct RangeDataTuple {
    RangeDataTuple () {}
    RangeDataTuple ( double median_dist, double bearing, double width ) : median_dist ( median_dist ), bearing ( bearing ), width ( width ) {}
    double median_dist;
    double bearing;
    double width;
    void printDataTuple () {
        std::cout << " median_dist : " << median_dist << ", bearing : " << bearing << " , width : " << width  << std::endl;
    }
};

struct LineSegmentDataTuple {
    LineSegmentDataTuple () {}
    LineSegmentDataTuple ( double x_frontal, double y_frontal, double x_distal, double y_distal ) : x_frontal ( x_frontal ), y_frontal ( y_frontal ), x_distal ( x_distal ), y_distal ( y_distal ) {}
    double x_frontal;
    double y_frontal;
    double x_distal;
    double y_distal;
    void printLineSegmentDataTuple () {
        std::cout << " x_frontal : " << x_frontal << ", y_frontal : " << y_frontal << " , x_distal : " << x_distal << " , y_distal : " << y_distal << std::endl;
    }
};



struct BoundaryError {
    BoundaryError ( Boundary boundary, double weight )
        : boundary_ ( boundary ), weight_ ( weight ) {}
    template <typename T>
    int isinside ( BoundaryShape boundary, T dx, T dy ) const {
        T d = T ( boundary.vert_2.bound_x * boundary.vert_3.bound_y - boundary.vert_2.bound_y * boundary.vert_3.bound_x );
        T w1 = ( dx* T ( boundary.vert_2.bound_y - boundary.vert_3.bound_y ) + dy * T ( boundary.vert_3.bound_x - boundary.vert_2.bound_x ) + d ) / d;
        T w2 = ( dx* T ( boundary.vert_3.bound_y ) - dy * T ( boundary.vert_3.bound_x ) ) / d;
        T w3 = ( -dx* T ( boundary.vert_2.bound_y ) + dy * T ( boundary.vert_2.bound_x ) ) / d;
        if ( w1 > T ( 0.0 ) && w1 < T ( 1.0 ) && w2 >  T ( 0.0 ) && w2 < T ( 1.0 ) && w3 >  T ( 0.0 ) && w3 <  T ( 1.0 ) ) {
            return 1;
        } else {
            return 0;
        }
    }
    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta,
                      T *residuals ) const {
        if ( dx_dy_dtheta[2] < boundary_.shape_1.vert_2.bound_theta && boundary_.shape_1.vert_3.bound_theta < dx_dy_dtheta[2] ) {
	    if ( isinside ( boundary_.shape_1, dx_dy_dtheta[0], dx_dy_dtheta[1] ) + isinside ( boundary_.shape_1, dx_dy_dtheta[0], dx_dy_dtheta[1] ) == 1 ) {
	      residuals[0] = T ( weight_ ) * T ( 0.0 );
	    } else {
            residuals[0] = T ( weight_ ) * T ( 1.0 );
	    }
        } else {
            residuals[0] = T ( weight_ ) * T ( 1.0 );
        }

        return true;
    }
    static ceres::CostFunction *Create ( Boundary boundary, double weight ) {
        return ( new ceres::AutoDiffCostFunction<BoundaryError, 1, 3> ( new BoundaryError ( boundary, weight ) ) );
    }
    Boundary boundary_;
    double weight_;
};

struct ClusterError {
    ClusterError ( RangeDataTuple cluster_tuple, double weight )
        : cluster_tuple ( cluster_tuple ), weight ( weight ) {}
    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta,
                      T *residuals ) const {
        T x = T ( cluster_tuple.median_dist ) * T ( cos ( cluster_tuple.bearing ) );
        T y = T ( cluster_tuple.median_dist ) * T ( sin ( cluster_tuple.bearing ) );
        T v1x = x - dx_dy_dtheta[0];
        T v1y = y - dx_dy_dtheta[1];
        residuals[0] = T ( weight ) * exp ( - ( v1x*v1x + v1y*v1y ) );
        return true;
    }
    static ceres::CostFunction *Create ( RangeDataTuple cluster_tuple, double weight ) {
        return ( new ceres::AutoDiffCostFunction<ClusterError, 1, 3> ( new ClusterError ( cluster_tuple, weight ) ) );
    }
    RangeDataTuple cluster_tuple;
    double weight;
};

struct LineSegmentError {
    LineSegmentError ( LineSegmentDataTuple line_tuple, double weight )
        : line_tuple ( line_tuple ), weight ( weight ) {}
    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta,
                      T *residuals ) const {
        T v1x = line_tuple.x_frontal - dx_dy_dtheta[0];
        T v1y = line_tuple.y_frontal - dx_dy_dtheta[1];
        T v2x = line_tuple.x_distal - dx_dy_dtheta[0];
        T v2y = line_tuple.y_distal - dx_dy_dtheta[1];

        T px = T ( line_tuple.x_distal - line_tuple.x_frontal );
        T py = T ( line_tuple.y_distal - line_tuple.y_frontal );
        T dAB = T ( px*px + py*py );
        T u = T ( ( dx_dy_dtheta[0] - line_tuple.x_frontal ) * px + ( dx_dy_dtheta[1] - line_tuple.y_frontal ) * py ) / T ( dAB );
        T projx = T ( line_tuple.x_frontal ) + u * px;
        T projy = T ( line_tuple.y_frontal ) + u * py;

        T v3x = projx - dx_dy_dtheta[0];
        T v3y = projy - dx_dy_dtheta[1];

        if ( ( T ( line_tuple.x_frontal ) - projx ) * ( T ( line_tuple.x_distal ) - projx ) < T ( 0.0 ) ) {
            residuals[0] = T ( weight ) * exp ( - ( v3x*v3x + v3y*v3y ) );
        } else {
            residuals[0] = T ( weight ) * exp ( -std::min ( v1x*v1x + v1y*v1y , v2x*v2x + v2y*v2y ) );
        }
        return true;
    }
    static ceres::CostFunction *Create ( LineSegmentDataTuple line_tuple, double weight ) {
        return ( new ceres::AutoDiffCostFunction<LineSegmentError, 1, 3> ( new LineSegmentError ( line_tuple, weight ) ) );
    }
    LineSegmentDataTuple line_tuple;
    double weight;
};


struct PanTiltChangeError {
    PanTiltChangeError ( double last_p, double last_t, double weight_p, double weight_t )
        : last_p ( last_p ), last_t ( last_t ), weight_p ( weight_p ), weight_t ( weight_t ) {}

    template <typename T>
    bool operator () ( const T *const p_t,
                       T *residuals ) const {
        residuals[0] = T ( weight_p ) * ( p_t[0] - T ( last_p ) );
        residuals[1] = T ( weight_t ) * ( p_t[1] - T ( last_t ) );
        return true;
    }

    static ceres::CostFunction *Create ( double last_p, double last_t, double weight_p, double weight_t ) {
        return ( new ceres::AutoDiffCostFunction<PanTiltChangeError, 2, 2> ( new PanTiltChangeError ( last_p, last_t, weight_p, weight_t ) ) );
    }
    double last_p;
    double last_t;
    double weight_p;
    double weight_t;
};


class DistanceErrorOnly : public ceres::SizedCostFunction<1,2>
{
public:
    DistanceErrorOnly ( const Eigen::Matrix4d tag_in_world, const double weight, const Eigen::Vector3d last_gt, const double dt ) :
        tag_in_world_ ( tag_in_world ), weight_ ( weight ), last_gt_ ( last_gt ), dt_ ( dt ) {}
    virtual ~DistanceErrorOnly() {}
    virtual bool Evaluate ( double const* const* parameters,
                            double* residuals,
                            double** jacobians ) const {

        const double vel = parameters[0][0];
        const double omega = parameters[0][1];
        const double term4 = sin ( last_gt_ ( 2,0 ) + dt_ * omega );
        const double term5 = cos ( last_gt_ ( 2,0 ) + dt_ * omega );
        const double term2 = last_gt_ ( 0,0 ) - tag_in_world_ ( 0,3 ) +  vel * term4 / omega - vel * sin ( last_gt_ ( 2,0 ) ) /omega;
        const double term3 = last_gt_ ( 1,0 ) - tag_in_world_ ( 1,3 ) - vel * term5 / omega +  vel * cos ( last_gt_ ( 2,0 ) ) /omega;
        const double term1 = 2 * sqrt ( term3 * term3  + term2 * term2 );
        const double term11 = term1 * omega;
        const double term12 = term1 * omega * omega;
        double x1 = last_gt_ ( 0,0 ) - vel / omega * sin ( last_gt_ ( 2,0 ) ) + vel / omega * sin ( last_gt_ ( 2,0 ) + omega * dt_ );
        double y1 = last_gt_ ( 1,0 ) + vel / omega * cos ( last_gt_ ( 2,0 ) ) - vel / omega * cos ( last_gt_ ( 2,0 ) + omega * dt_ );

        residuals[0] = sqrt ( ( x1 - tag_in_world_ ( 0,3 ) ) * ( x1 - tag_in_world_ ( 0,3 ) ) +
                              ( y1 - tag_in_world_ ( 1,3 ) ) * ( y1 - tag_in_world_ ( 1,3 ) ) );

        if ( !jacobians ) {
            return true;
        }
        double* jacobian = jacobians[0];
        if ( !jacobian ) {
            return true;
        }

        jacobian[0] = ( 2 * ( term5 - cos ( last_gt_ ( 2,0 ) ) ) * term3 + 2 * ( sin ( last_gt_ ( 2,0 ) ) - term4 ) * term2 ) / -term11;
        jacobian[1] = ( 2 * ( vel * sin ( last_gt_ ( 2,0 ) ) - vel * term4 + dt_ * vel * term5 * omega ) * term2 + 2 * ( vel * term5 - vel * cos ( last_gt_ ( 2,0 ) ) + dt_ * vel * term4 * omega ) * term3 ) / term12;

        return true;
    }

private:
    const Eigen::Matrix4d tag_in_world_;
    const double weight_;
    const Eigen::Vector3d last_gt_;
    const double dt_;
};


struct xyError {
    xyError ( const Eigen::Vector4d tag_in_body, const double weight, const Eigen::Vector3d last_gt, const double dt ) :
        tag_in_body_ ( tag_in_body ), weight_ ( weight ), last_gt_ ( last_gt ), dt_ ( dt ) {}

    template <typename T>
    bool operator() ( const T *const dx_dy_dtheta,
                      T *residuals ) const {

        if ( tag_in_body_ ( 0,0 ) != tag_in_body_ ( 0,0 ) ) {
            residuals[0] = T ( 0 );
            return true;
        }
        Eigen::Matrix<T, 4, 1> Ttag_in_body = tag_in_body_.cast<T>();
        Eigen::Matrix<T, 3, 1> Tlast_gt = last_gt_.cast<T>();

        residuals[0] = T ( weight_ ) * T ( dx_dy_dtheta[0] - Ttag_in_body ( 0,0 ) );
        residuals[1] = T ( weight_ ) * T ( dx_dy_dtheta[1] - Ttag_in_body ( 1,0 ) );
        residuals[2] = T ( 0.0 );
        return true;
    }
    static ceres::CostFunction *Create ( const Eigen::Vector4d tag_in_body, const double weight, const Eigen::Vector3d last_gt, const double dt ) {
        return ( new ceres::AutoDiffCostFunction<xyError, 3, 3> ( new xyError ( tag_in_body, weight, last_gt, dt ) ) );
    }
    const Eigen::Vector4d tag_in_body_;
    const double weight_;
    const Eigen::Vector3d last_gt_;
    const double dt_;
};

struct DistanceError {
    DistanceError ( const Eigen::Vector4d tag_in_body, const double weight, const Eigen::Vector3d last_gt, const double dt ) :
        tag_in_body_ ( tag_in_body ), weight_ ( weight ), last_gt_ ( last_gt ), dt_ ( dt ) {}

    template <typename T>
    bool operator() ( const T *const vel_omega,
                      T *residuals ) const {

        T vel = vel_omega[0];
        T omega = vel_omega[1];

        if ( tag_in_body_ ( 0,0 ) != tag_in_body_ ( 0,0 ) ) {
            residuals[0] = T ( 0 );
            residuals[1] = T ( 0 );
            return true;
        }
        Eigen::Matrix<T, 4, 1> Ttag_in_body = tag_in_body_.cast<T>();
        Eigen::Matrix<T, 3, 1> Tlast_gt = last_gt_.cast<T>();

        T x1 = vel / omega * sin ( omega * T ( dt_ ) );
        T y1 = vel / omega - vel / omega * cos ( omega * T ( dt_ ) );

        residuals[0] = T ( x1 - Ttag_in_body ( 0,0 ) );
        residuals[1] = T ( y1 - Ttag_in_body ( 1,0 ) );
//         residuals[2] = T ( weight_ ) * T ( omega * dt_ - atan2 ( Ttag_in_body ( 1,0 ), Ttag_in_body ( 0,0 ) ) );
        residuals[2] = T ( weight_ ) * T ( 0.0 );

//         std::cout << "Xdesired: " << Ttag_in_world ( 0,3 ) << std::endl;
//         std::cout << "Ydesired: " << Ttag_in_world ( 1,3 ) << std::endl;
//         std::cout << "X1: " << x1 << std::endl;
//         std::cout << "Y1: " << y1 << std::endl;
        return true;
    }
    static ceres::CostFunction *Create ( const Eigen::Vector4d tag_in_body, const double weight, const Eigen::Vector3d last_gt, const double dt ) {
        return ( new ceres::AutoDiffCostFunction<DistanceError, 3, 2> ( new DistanceError ( tag_in_body, weight, last_gt, dt ) ) );
    }
    const Eigen::Vector4d tag_in_body_;
    const double weight_;
    const Eigen::Vector3d last_gt_;
    const double dt_;
};

struct ProjectionErrorPTOnly {
    ProjectionErrorPTOnly ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight )
        : target_in_cam ( target_in_cam ), K ( K ), weight ( weight ), cam_in_body_old ( cam_in_body_old ) {}

    template <typename T>
    bool operator() ( const T *const p_t,
                      const T *const dx_dy_dtheta,
                      T *residuals ) const {

        T cam_in_body_new[16];
        T sp = sin ( p_t[0] );
        T st = sin ( p_t[1] );
        T cp = cos ( p_t[0] );
        T ct = cos ( p_t[1] );
        cam_in_body_new[0] = -sp;
        cam_in_body_new[1] = st * cp;
        cam_in_body_new[2] = cp * ct;
        cam_in_body_new[3] = T ( 0.0 );
        cam_in_body_new[4] = -cp;
        cam_in_body_new[5] = -st * sp;
        cam_in_body_new[6] = -ct * sp;
        cam_in_body_new[7] = T ( 0.0 );
        cam_in_body_new[8] = T ( 0.0 );
        cam_in_body_new[9] = -ct;
        cam_in_body_new[10] = st;
        cam_in_body_new[11] = T ( 0.34 );
        cam_in_body_new[12] = T ( 0.0 );
        cam_in_body_new[13] = T ( 0.0 );
        cam_in_body_new[14] = T ( 0.0 );
        cam_in_body_new[15] = T ( 1.0 );

        Eigen::Map<const Eigen::Matrix < T, 4, 4, Eigen::RowMajor> > Tcam_in_body_new ( cam_in_body_new );
        Eigen::Matrix < T, 4, 4> Tbody_in_cam_new = Tcam_in_body_new.inverse();

        Eigen::Matrix<T, 3, 4> TK = K.cast <T> ();
        Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
        Eigen::Matrix<T, 4, 1> Ttarget_in_cam = target_in_cam.cast<T>();

        T new_body_in_old_body[16];

        T sdtheta = sin ( dx_dy_dtheta[2] );
        T cdtheta = cos ( dx_dy_dtheta[2] );
        T dx = dx_dy_dtheta[0];
        T dy = dx_dy_dtheta[1];

        new_body_in_old_body[0] = cdtheta;
        new_body_in_old_body[1] = -sdtheta;
        new_body_in_old_body[2] =  T ( 0.0 );
        new_body_in_old_body[3] = dx;
        new_body_in_old_body[4] = sdtheta;
        new_body_in_old_body[5] = cdtheta;
        new_body_in_old_body[6] = T ( 0.0 );
        new_body_in_old_body[7] = dy;
        new_body_in_old_body[8] = T ( 0.0 );
        new_body_in_old_body[9] = T ( 0.0 );
        new_body_in_old_body[10] = T ( 1.0 );
        new_body_in_old_body[11] = T ( 0.0 );
        new_body_in_old_body[12] = T ( 0.0 );
        new_body_in_old_body[13] = T ( 0.0 );
        new_body_in_old_body[14] = T ( 0.0 );
        new_body_in_old_body[15] = T ( 1.0 );

        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > Tnew_body_in_old_body ( new_body_in_old_body );

        Eigen::Matrix<T, 3, 1> new_projection = TK * ( Tbody_in_cam_new * Tnew_body_in_old_body.inverse() * Tcam_in_body_old * Ttarget_in_cam ) / Ttarget_in_cam ( 2 , 0 );

        residuals[0] = T ( weight ) * ( new_projection ( 0 , 0 ) - TK ( 0 , 2 ) );
        residuals[1] = T ( weight ) * ( new_projection ( 1 , 0 ) - TK ( 1 , 2 ) );
        return true;
    }

    static ceres::CostFunction *Create ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight ) {
        return ( new ceres::AutoDiffCostFunction<ProjectionErrorPTOnly, 2, 2, 3> ( new ProjectionErrorPTOnly ( target_in_cam, K, cam_in_body_old, weight ) ) );
    }

    Eigen::Vector4d target_in_cam;
    Eigen::Matrix< double, 3, 4 > K;
    Eigen::Matrix4d cam_in_body_old;
    double weight;
};



struct ProjectionError {
    ProjectionError ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight )
        : target_in_cam ( target_in_cam ), K ( K ), weight ( weight ), cam_in_body_old ( cam_in_body_old ) {}

    template <typename T>
    bool operator() ( const T *const p_t,
                      const T *const dx_dy_dtheta_vel_omega,
                      T *residuals ) const {
        T cam_in_body_new[16];
        T sp = sin ( p_t[0] );
        T st = sin ( p_t[1] );
        T cp = cos ( p_t[0] );
        T ct = cos ( p_t[1] );
        cam_in_body_new[0] = -sp;
        cam_in_body_new[1] = st * cp;
        cam_in_body_new[2] = cp * ct;
        cam_in_body_new[3] = T ( 0.0 );
        cam_in_body_new[4] = -cp;
        cam_in_body_new[5] = -st * sp;
        cam_in_body_new[6] = -ct * sp;
        cam_in_body_new[7] = T ( 0.0 );
        cam_in_body_new[8] = T ( 0.0 );
        cam_in_body_new[9] = -ct;
        cam_in_body_new[10] = st;
        cam_in_body_new[11] = T ( 0.34 );
        cam_in_body_new[12] = T ( 0.0 );
        cam_in_body_new[13] = T ( 0.0 );
        cam_in_body_new[14] = T ( 0.0 );
        cam_in_body_new[15] = T ( 1.0 );
        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > eigen_cam_in_body_new ( cam_in_body_new );

        T new_body_in_old_body[16];

        T sdtheta = sin ( dx_dy_dtheta_vel_omega[2] );
        T cdtheta = cos ( dx_dy_dtheta_vel_omega[2] );
        T dx = dx_dy_dtheta_vel_omega[0];
        T dy = dx_dy_dtheta_vel_omega[1];

        new_body_in_old_body[0] = cdtheta;
        new_body_in_old_body[1] = -sdtheta;
        new_body_in_old_body[2] =  T ( 0.0 );
        new_body_in_old_body[3] = dx;
        new_body_in_old_body[4] = sdtheta;
        new_body_in_old_body[5] = cdtheta;
        new_body_in_old_body[6] = T ( 0.0 );
        new_body_in_old_body[7] = dy;
        new_body_in_old_body[8] = T ( 0.0 );
        new_body_in_old_body[9] = T ( 0.0 );
        new_body_in_old_body[10] = T ( 1.0 );
        new_body_in_old_body[11] = T ( 0.0 );
        new_body_in_old_body[12] = T ( 0.0 );
        new_body_in_old_body[13] = T ( 0.0 );
        new_body_in_old_body[14] = T ( 0.0 );
        new_body_in_old_body[15] = T ( 1.0 );

        Eigen::Map<const Eigen::Matrix<T, 4, 4, Eigen::RowMajor> > eigen_new_body_in_old_body ( new_body_in_old_body );


        Eigen::Matrix<T, 3, 4> TK = K.cast <T> ();
        Eigen::Matrix<T, 4, 4> Tcam_in_body_old = cam_in_body_old.cast<T> ();
        Eigen::Matrix<T, 4, 1> Ttarget_in_cam = target_in_cam.cast<T>();

        Eigen::Matrix<T, 3, 1> new_projection = TK * ( eigen_cam_in_body_new.inverse() * eigen_new_body_in_old_body.inverse()  * Tcam_in_body_old * Ttarget_in_cam );

        residuals[0] = T ( weight ) * ( new_projection ( 0 , 0 ) - TK ( 0 , 2 ) );
        residuals[1] = T ( weight ) * ( new_projection ( 1 , 0 ) - TK ( 1 , 2 ) );
        return true;
    }

    static ceres::CostFunction *Create ( Eigen::Vector4d target_in_cam, Eigen::Matrix< double, 3, 4 > K, Eigen::Matrix4d cam_in_body_old, double weight ) {
        return ( new ceres::AutoDiffCostFunction<ProjectionError, 2, 5, 2> ( new ProjectionError ( target_in_cam, K, cam_in_body_old, weight ) ) );
    }

    Eigen::Vector4d target_in_cam;
    Eigen::Matrix< double, 3, 4 > K;
    Eigen::Matrix4d cam_in_body_old;
    double weight;
};







