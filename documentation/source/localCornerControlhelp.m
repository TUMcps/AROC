%% localCornerControl
% Solve optimal control problem for the corner states
% of the parallelotope
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [uTotal, xt]= localCornerControl(system,xf,vert,h,Q,R,steps,lenHorizon,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function solves an optimal control problem for all vertices
% of the initial parallelotope (see Lines 6-8 of Alg. 1 in [1]).
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
%            process
%            (dimension: [nx, 1])
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
%            length of one timestep of the multiple shooting
%            algorithm
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
%            a structure containing following options
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .parallel
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            boolean value that determines if parallel
%            computing should be used or not (0 or 1)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .useAcado
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use ACADO toolbox for solving the optimal
%            control problem. Fmincon is used otherwise
%            (0 or 1)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .extHorizon.decay
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            decay function for the objective
%            function of the optimization problem
%            with extended optimization horizon
%            [<tt>'uniform'</tt> / <tt>'fall'</tt> / <tt>'fall+end'</tt> /;
%            <tt>'fallLinear'</tt> / <tt>'fallLinear+End'</tt> /;
%            <tt>'fallEqDiff'</tt> / <tt>'FallEqDiff+End'</tt> /;
%            <tt>'rise'</tt> / <tt>'quad'</tt> /  <tt>'riseLinear'</tt> /;
%            <tt>'riseEqDiff'</tt> / <tt>'end'</tt>]
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
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
%            uTotal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            optimal control inputs for the corner trajectories
%            (dimension: [nu,2^nx,steps*lenHorizon])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            xt
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            resulting corner trajectories
%            (dimension: [nx,2^nx,steps*lenHorizon])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <optimalControlhelp.html |optimalControl|>, <optimalControlFminconhelp.html |optimalControlFmincon|>
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
