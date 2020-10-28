%% optimalControlFmincon
% Solve optimal control problem with FMINCON
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [u,x] = optimalControlFmincon(system,xf,x0,h,Q,R,steps, ... <br>
% u0,maxFunEvals,lenHorizon,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function solves an optimal control problem using a multiple
% shooting algorithm and FMINCON
%
%% Input Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            system
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            object containing the system dynamics (class:;
%            nonlinearSys)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            xf
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            desired final state at the end of the control
%            process (dimension: [nx, 1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial state (dimension: [nx, 1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            vert
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vertices of the initial parallelotope.
%            (dimension: [nu,2^nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            h
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            length of one timestep of the corner trajectories
%            during which the control input is constant
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            weighting matrix for the final state of the optimal
%            control problem
%            (dimension: [nx,nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            weighting matrix for the input term of the optimal
%            control problem
%            (dimension: [nu,nu])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            steps
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of intermediate timesteps of the corner
%            trajectories during one timestep of the center
%            trajectory
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input value for computing the initial point for
%            fmincon by integrating with Runge-Kutta
%            (scalar value)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            maxFunEvals
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Maximum number of function evaluations for fmincon
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            lenHorizon
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            length of the optimization horizon in center
%            trajectory time steps
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            structure containing user defined options for the
%            algorithm
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% Output Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            optimal control input
%            (dimension: [nu,steps*lenHorizon])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            resulting state trajectories
%            (dimension: [nx,steps*lenHorizon])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <optimalControlFminconhelp.html |optimalControlFmincon|>
%
%% References
% * *[1] Schuermann et al. (2017)*, Convex interpolation control with
% formal guarantees for disturbed and constrained nonlinear
% systems
%

%%
% <html>
%   <hr>
%   <p class="copy">&copy; 2018-2020 I6 Technische Universit&auml;t M&uuml;nchen
%        <tt class="minicdot">&#149;</tt>
%        <a href="https://tumcps.github.io/AROC/">Website</a>
%        <tt class="minicdot">&#149;</tt>
%        <a href="file:txts/LICENSE.txt">License</a>
%   </p>
% <div>
% <table>
%  <tr>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%             <img src="img/logoAroc.png" alt="logoAroc" height="40px">
%      </td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoCora.png" alt="logoCora" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoChair.png" alt="logoChair" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoTum.png" alt="logoTum" height="40px"></td>
%  </tr>
% </table>
% </div>
% </html>
