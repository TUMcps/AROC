%% computeLinearController
% implementation of the linear control law
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [zonoBig,reachSet,controlLawParam] = ... <br>
% computeLinearController(dynamic,zono,zonoR,uCorner,uCenter,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function implements the control law that uses a linear
% approximation of the convex combinations of the corner inputs. The
% reachable set is computed for one time step of the center
% trajectory, which consists of multiple steps of the corner
% trajectories. The function implements Alg. 2 in [1].
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
%            dynamic
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
%            zono
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial zonotope (class zonotope or polyZonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            zonoR
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            parallelotope which is an overapproximation of the
%            initial zonotope zono (class zonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uCorner
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            optimal control inputs for the corner trajectories
%            (dimension: [nu,nx^2,Opts.Ninter])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uCenter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            optimal control inputs for the center trajectory
%            (dimension: [nx, Opts.Nc])
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
%            zonoBig
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            extended zonotope that results from the
%            reachability analysis. This zonotope still
%            includes the auxiliary states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            reachSet
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell array containing the reachable sets of all
%            intermediate time steps
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            controlLawParam
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            parameters of the computed control law
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <computeExactControllerhelp.html |computeExactController|>
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
