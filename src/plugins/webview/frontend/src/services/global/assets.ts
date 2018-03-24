
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Inject, Injectable} from '@angular/core';
import {MatIconRegistry} from '@angular/material';
import {DomSanitizer} from '@angular/platform-browser';

@Injectable()
export class AssetsService {
  private readonly assetsPath_ = 'assets/images';
  private readonly appLogoSvg_ = 'fawkes-logo.svg';
  private readonly appLogoTextSvg_ = 'webview-logo.svg';
  private readonly appLogoIcon_ = 'ff-logo';
  private readonly appLogoTextIcon_ = 'ff-logo-text';

  constructor(
    @Inject(MatIconRegistry) private readonly iconRegistry_: MatIconRegistry,
    @Inject(DomSanitizer) private readonly sanitizer_: DomSanitizer)
  {
    iconRegistry_.addSvgIcon(
      this.appLogoIcon_,
      sanitizer_.bypassSecurityTrustResourceUrl(`${this.assetsPath_}/${this.appLogoSvg_}`));
    iconRegistry_.addSvgIcon(
      this.appLogoTextIcon_,
      sanitizer_.bypassSecurityTrustResourceUrl(`${this.assetsPath_}/${this.appLogoTextSvg_}`));
  }

  getAppLogo(): string {
    return this.appLogoIcon_;
  }

  getAppLogoText(): string {
    return this.appLogoTextIcon_;
  }
}
