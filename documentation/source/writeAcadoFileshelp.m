%% writeAcadoFiles
% Generate files for ACADO Toolbox
%
%% Syntax
%
% <html>
%        <div class="syntax">
% writeAcadoFiles(path,benchmark,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function generates and compiles the files containing the
% system dynamics, which are needed for solving optimal control
% problems with the ACADO toolbox
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
%            path
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            path of the root directory for the Convex Interpolation
%            Control Algorithm
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            benchmark
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            name of the selected benchmark
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
%            .extHorizon.active
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            use extended optimization horizon for
%            optimal control problems (optional)
%            [&#123;false&#125; / true]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .extHorizon.horizon
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            length of the extended optimization
%            horizon in center trajectory time steps
%            (optional)
%            [&#123;<tt>'all'</tt>&#125; / positive integer]
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
%            (optional)
%            [&#123;<tt>'uniform'</tt>&#125; / <tt>'fall'</tt> / <tt>'fall+end'</tt> /;
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
%% See Also
% <acadoConvertDynamicshelp.html |acadoConvertDynamics|>
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
