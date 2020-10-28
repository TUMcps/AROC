%% optimalControlReachsetMPC
% solve an optimal control problem
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [param,res] = optimalControlReachsetMPC(x0,J,L,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function solves the optimal control problem as defined in
% Eq. (7) in [1] to determine a feasible reference trajectory.
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
%            x0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial state of the considered reference trajectory part
%            (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            L
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            value of the objective function (=cost) of the previous
%            solution
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            J
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            summed distance of the points from the previous solution to
%            the terminal region
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
%            .funHandle
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the dynamic function
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nx
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of system states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nu
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of system inputs
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .uMax
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the input constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .uMin
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            lower bound for the input constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .xf
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            goal state
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of time steps for the prediction
%            horizon. Prediction horizon: Opts.N * Opts.dT
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .dT
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time step. Prediction horizon: Opts.N * Opts.dT
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function
%            (center trajectory)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function
%            (center trajectory)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .termReg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            terminal region around the steady state xf
%            (class: mptPolytope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .alpha
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            contraction rate for the contraction constraint
%            [&#123;0.1&#125; / alpha > 0]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .maxIter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            maximum number of iterations for the optimal
%            control problem [&#123;10&#125; / positive integer]
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
%            param
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
%            .xc
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference trajectory (dimension: [nx,N+1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .uc
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference trajectory control inputs
%            (dimension: [nu,N])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .J
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            summed distance of the reference trajectory points
%            from the terminal region
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .L
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            value of the objective function (=cost) of the
%            reference trajectory
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            res
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            flag that specifies if the computed reference
%            trajectory is feasible (0 or 1)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <optimalControlhelp.html |optimalControl|>, <reachsetMPChelp.html |reachsetMPC|>
%
%% References
% * *[1] Schuermann et al. (2018)*, Reachset Model Predictive Control
% for Disturbed Nonlinear Systems
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
