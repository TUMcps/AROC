%% optimizationBasedControl
% Implementation of a optimazation based control
% algorithm
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [objContr,res] = optimizationBasedControl(benchmark,Param) <br>
% [objContr,res] = optimizationBasedControl(benchmark,Param,Opts) <br>
% [objContr,res] = optimizationBasedControl(benchmark,Param,Opts,Post) <br>
%        </div>
% </html>
%
%% Description
% Offline-phase computations for a control algorithm that is based on
% the optimization over reachable sets
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
%            name of the considered benchmark model (see;
%            "aroc/benchmarks/...")
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Param
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the benchmark parameters
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial set of states (class: interval)
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
%            .tFinal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final time after which the goal state should be
%            reached
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .U
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of admissible control inputs (class:;
%            interval)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .W
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of uncertain disturbances (class: interval)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .X
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of state constraints (class: mptPolytope)
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
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the algorithm settings
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of time steps
%            [&#123;5&#125; / positive integer]
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
%            number of reachability steps during one
%            time step (optimization)
%            [&#123;10&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .reachStepsFin
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of reachability steps during one
%            time step (final reachable set computation)
%            [&#123;100&#125; / positive integer]
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
%            maximum number of iterations for
%            optimization with fmincon
%            [&#123;15&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .bound
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            scaling factor between upper and lower
%            bound of the weigthing matrices
%            [&#123;1000&#125; / positive scalar]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function of
%            optimal control problem (dimension:[nx,nx])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function of
%            optimal control problem (dimension:[nu,nu])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .refTraj.x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            user provided reference trajectory
%            (dimension: [nx,Opts.N + 1])
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%            -.refTraj.u     inputs for the user provided reference
%            trajectory (dimension: [nu,Opts.N])
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.alg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachability algorithm (nonlinear)
%            [&#123;<tt>'lin'</tt>&#125; / <tt>'poly'</tt>]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.linAlg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reachability algorithm (linear)
%            [&#123;<tt>'standard'</tt>&#125; / <tt>'fromStart'</tt> /;
%            <tt>'wrapping-free'</tt> / <tt>'adap'</tt>]
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
%            the nonlinear function
%            [&#123;2&#125; / 3]
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
%            [&#123;50&#125; / positive integer]
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
%            [&#123;5&#125; / positive integer]
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
%            [&#123;30&#125; / positive integer]
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .cora.error
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            uppper-bound for Hausdorff distance
%            (for cora.linAlg = <tt>'adap'</tt> only)
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
%            Post
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the postprocessing function that is
%            used to compute the occupancy set
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
%            objContr
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            resulting controller storing the data computed during
%            the offline phase (class: objOptBasedContr)
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
%            results object storing the computed reachable set and
%            the center trajectory
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <objOptBasedContrhelp.html |objOptBasedContr|>
%
%% References
% * *[1] Schuermann et al. (2017)*, Optimal Control of Sets of
% Solutions to Formally Guarantee Constraints of Disturbed
% Linear Systems
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
