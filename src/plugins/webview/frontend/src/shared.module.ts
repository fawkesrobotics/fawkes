
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {CommonModule} from '@angular/common';
import {NgModule} from '@angular/core';
import {FlexLayoutModule} from '@angular/flex-layout';
import {FormsModule} from '@angular/forms';
import {MAT_TOOLTIP_DEFAULT_OPTIONS, MatButtonModule, MatCardModule, MatChipsModule, MatDialogModule, MatDividerModule, MatFormFieldModule, MatGridListModule, MatIconModule, MatInputModule, MatMenuModule, MatPaginatorModule, MatProgressSpinnerModule, MatRadioModule, MatSelectModule, MatSidenavModule, MatSliderModule, MatSlideToggleModule, MatSortModule, MatTableModule, MatTabsModule, MatToolbarModule, MatTooltipModule, MatExpansionModule, MatSnackBarModule, MatCheckboxModule} from '@angular/material';

//import {PipesModule} from './common/pipes/module';
//import {KD_TOOLTIP_DEFAULT_OPTIONS} from './index.config';

const SHARED_DEPENDENCIES = [
  // Angular imports
  CommonModule,
  FormsModule,

  // Material imports
  MatButtonModule,
  MatCardModule,
  MatDividerModule,
  MatGridListModule,
  MatFormFieldModule,
  MatIconModule,
  MatInputModule,
  MatProgressSpinnerModule,
  MatRadioModule,
  MatSidenavModule,
  MatToolbarModule,
  MatTooltipModule,
  MatSliderModule,
  MatDialogModule,
  MatSlideToggleModule,
  MatChipsModule,
  MatTableModule,
  MatSortModule,
  MatPaginatorModule,
  MatTabsModule,
  MatMenuModule,
  MatSelectModule,
  MatExpansionModule,
  MatSnackBarModule,
  MatCheckboxModule,

  // Other 3rd party modules
  FlexLayoutModule,

  // Custom application modules
  //PipesModule,
];

@NgModule({
  imports: SHARED_DEPENDENCIES,
  exports: SHARED_DEPENDENCIES,
  providers: [
    {
      provide: MAT_TOOLTIP_DEFAULT_OPTIONS,
      useValue: MAT_TOOLTIP_DEFAULT_OPTIONS,
    },
  ],
})
export class SharedModule {}
