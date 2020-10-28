%% initializeDynamics
% initialize the class object that contains the system
% dynamics
%
%% Syntax
%
% <html>
%        <div class="syntax">
% dynamics = initializeDynamics(benchmark, nExtended ,nParam, Opts) <br>
%        </div>
% </html>
%
%% Description
% This function creates the class object that represent the benchmark
% model. Depending on the chosen controller, the object is either of
% class nonlinearParamSys, nonlinearSys or nonlinearContrSys. All
% these classes are part of the CORA toolbox.
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
%            benchmark
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            name of the selected benchmark model
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nExtended
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            dimensions of the extended system dynimic with
%            auxiliary states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nParam
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of parameter for the system
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
%            dynamics
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            resulting object. (class nonlinerSys,;
%            nonlinParamSys)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>
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
%        <a href="http://cora.in.tum.de">Website</a>
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
