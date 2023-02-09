#!/bin/bash
# Copyright 2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

set -e
IMAGE_NAME=$1
IMAGE_TAG=$2
IMAGE_REPO=$3

aws codebuild start-build \
        --project-name generate_changelog \
        --environment-variables-override name=image_name,value=$IMAGE_NAME \
                                         name=image_tag,value=$IMAGE_TAG \
                                         name=image_repository,value=$IMAGE_REPO