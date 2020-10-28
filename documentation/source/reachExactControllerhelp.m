%% reachExactController
% reachabilty analysis for the exact control law
% implementation
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [Rfinal,reachSet] = reachExactController(dynamic,R0,param,Opts) <br>
%        </div>
% </html>
%
%% Description
% Computes the reachable sets at the end of one timestep of the
% center trajectory as well as all reachable sets during this
% timestep
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
%            nonlinParamSys)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            R0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            extended initial zonotope, which contains the state
%            as well as the auxiliary states (dimension: 2*nx,;
%            class zonotope or polyZonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            param
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            parameter vector. Containts the optimal inputs for
%            the vertices of the parallelotope, which are needed
%            for the online implementation of the control law
%            (dimension: [nu*(2^nx),1])
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
%            Rfinal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final zonotope at the end of reachability analysis
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
