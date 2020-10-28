%% settings_reachsetMPC_stirredTankReactor_traj2
% algorithm settings for
% the stirred tank reactor
% benchmark
%
%% Syntax
%
% <html>
%        <div class="syntax">
% Opts = settings_reachsetMPC_stirredTankReactor_traj2() <br>
%        </div>
% </html>
%
%% Description
% Algorithm settings and parameter for the Reachset Model Predictive
% Control algorithm for the stirred tank reactor benchmark
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
%            .tOpt
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final time for the optimization
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
%            number of time steps
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .reachSteps
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of reachability steps during one time
%            step [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .U_
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            tightened set of admissible control inputs
%            (class: interval)
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
%            .Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function of
%            optimal control problem (reference trajectory)
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
%            input weighting matrix for the cost function of
%            optimal control problem (reference trajectory)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Qlqr
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function of
%            the LQR approach (tracking controller)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Rlqr
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function of
%            the LQR approach (tracking controller)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .realTime
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            flag specifying if real time computation time
%            constraints are considered (Opts.realTime = 1)
%            or not (Opts.realTime = 0) [&#123;true&#125; / boolean]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .tComp
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time allocated to perform the computations for
%            the optimizations (0 < tComp < tOpt/N).
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
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.alg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachability algorithm that is used
%            [&#123;<tt>'lin'</tt>&#125; / <tt>'poly'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.tensorOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            taylor order for the abstraction of
%            the nonlinear function [&#123;2&#125;/ 3]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.taylorTerms
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            taylor order for computing e^At
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.zonotopeOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            [&#123;5&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.errorOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            before comp. the abstraction error
%            [&#123;3&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.intermediateOrder
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            upper bound for the zonotope order
%            during internal computations
%            [&#123;3&#125; / positive integer]
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
% <reachsetMPChelp.html |reachsetMPC|>, <stirredTankReactorhelp.html |stirredTankReactor|>
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
